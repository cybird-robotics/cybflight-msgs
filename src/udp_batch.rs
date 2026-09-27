//! Downlink datagram envelope between the ESP bridge and the ground station.
//!
//! The STM32 → ESP32 UART carries COBS-delimited wire frames
//! (`[msg_id][seq][payload…]`). The bridge used to forward each frame as its
//! own UDP datagram, which at ~250 frames/s is mostly per-packet overhead on a
//! half-duplex WiFi radio that must also *receive* RTK corrections. Instead it
//! packs every frame of one UART burst into a single datagram:
//!
//! ```text
//! [0x00 MARKER][VERSION]
//! [uart_errors: u16 LE][dl_drops: u16 LE][rtcm_rx: u16 LE][rtcm_drops: u16 LE]
//! ([len: u8][frame: len bytes])*
//! ```
//!
//! * `MARKER` is 0, a value [`crate::wire::msg_id`] reserves and never
//!   assigns, so a receiver tells a batch from a legacy single-frame datagram
//!   by its first byte alone. Both forms are accepted by [`classify`](crate::udp_batch::classify).
//! * The four counters are the bridge's own cumulative, wrapping health
//!   counters — UART0 receive errors, telemetry frames the bridge dropped, and
//!   RTCM datagrams received / dropped — so the ground station can tell loss
//!   inside the bridge apart from loss on the air, and watch the RTCM path,
//!   without a USB log attached.
//! * Each frame is `2 ..= MAX_FRAME_SIZE` bytes, so one length byte suffices.
//!
//! A bridge belongs to one ground station at a time. It answers any other
//! sender's uplink with a **busy notice**, `[0x00 MARKER][KIND_BUSY][owner
//! IPv4: 4 bytes]`, at most once a second, so a second ground station started
//! by mistake can tell its operator which PC owns the drone. [`classify`](crate::udp_batch::classify)
//! reports it as [`Datagram::Busy`](crate::udp_batch::Datagram::Busy); a receiver that predates it sees an
//! unsupported version and ignores it.
//!
//! `no_std`, no allocation: the same code runs on the bridge and the host.

use crate::wire::{FRAME_HEADER_SIZE, MAX_FRAME_SIZE};

/// First byte of a batch datagram (a reserved, never-assigned msg_id).
pub const MARKER: u8 = crate::wire::msg_id::RESERVED_BATCH_MARKER;
/// Envelope version this crate reads and writes.
pub const VERSION: u8 = 1;
/// Marker + version + four u16 counters.
pub const HEADER_LEN: usize = 10;
/// Largest datagram a writer should produce: fits a 1500 B Ethernet MTU with
/// IP/UDP headers and 802.11 encapsulation to spare, so it is never fragmented.
pub const MAX_DATAGRAM: usize = 1400;

/// Second byte of a busy notice, in place of [`VERSION`].
pub const KIND_BUSY: u8 = 0x80;
/// Length of a busy notice: marker, kind, owner IPv4.
pub const BUSY_LEN: usize = 6;

/// A busy notice naming `owner`, the ground station the bridge belongs to.
pub fn busy_notice(owner: [u8; 4]) -> [u8; BUSY_LEN] {
    [MARKER, KIND_BUSY, owner[0], owner[1], owner[2], owner[3]]
}

/// Bridge-side cumulative counters carried in every batch header. Each wraps
/// at `u16::MAX`; consumers take wrapping deltas between datagrams.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct BatchHeader {
    /// UART0 (STM32 link) receive errors: FIFO overflows, framing, parity.
    pub uart_errors: u16,
    /// Complete telemetry frames the bridge discarded (queue full, bad COBS).
    pub dl_drops: u16,
    /// RTCM correction datagrams received from the ground station.
    pub rtcm_rx: u16,
    /// RTCM datagrams dropped before reaching the GNSS receiver's UART.
    pub rtcm_drops: u16,
}

impl BatchHeader {
    fn write(&self, out: &mut [u8]) {
        out[0] = MARKER;
        out[1] = VERSION;
        out[2..4].copy_from_slice(&self.uart_errors.to_le_bytes());
        out[4..6].copy_from_slice(&self.dl_drops.to_le_bytes());
        out[6..8].copy_from_slice(&self.rtcm_rx.to_le_bytes());
        out[8..10].copy_from_slice(&self.rtcm_drops.to_le_bytes());
    }

    fn read(d: &[u8]) -> Self {
        let u = |i: usize| u16::from_le_bytes([d[i], d[i + 1]]);
        Self {
            uart_errors: u(2),
            dl_drops: u(4),
            rtcm_rx: u(6),
            rtcm_drops: u(8),
        }
    }
}

/// Packs frames into one batch datagram inside a caller-owned buffer.
pub struct BatchWriter<'a> {
    buf: &'a mut [u8],
    len: usize,
    frames: usize,
}

impl<'a> BatchWriter<'a> {
    /// Start a batch in `buf`. Usable capacity is `min(buf.len(), MAX_DATAGRAM)`.
    ///
    /// # Panics
    /// If `buf` cannot hold the header.
    pub fn new(buf: &'a mut [u8], hdr: BatchHeader) -> Self {
        assert!(
            buf.len() >= HEADER_LEN,
            "batch buffer smaller than its header"
        );
        hdr.write(buf);
        Self {
            buf,
            len: HEADER_LEN,
            frames: 0,
        }
    }

    /// Discard the frames and start over with a fresh header.
    pub fn reset(&mut self, hdr: BatchHeader) {
        hdr.write(self.buf);
        self.len = HEADER_LEN;
        self.frames = 0;
    }

    fn capacity(&self) -> usize {
        self.buf.len().min(MAX_DATAGRAM)
    }

    /// Whether `frame` is a legal entry and would fit right now.
    pub fn fits(&self, frame: &[u8]) -> bool {
        valid_frame_len(frame.len()) && self.len + 1 + frame.len() <= self.capacity()
    }

    /// Append one frame. Returns `false` — leaving the batch unchanged — when
    /// the frame is not a legal entry (`2..=MAX_FRAME_SIZE` bytes) or does
    /// not fit; the caller then sends this batch and starts another.
    pub fn try_push(&mut self, frame: &[u8]) -> bool {
        if !self.fits(frame) {
            return false;
        }
        self.buf[self.len] = frame.len() as u8;
        self.buf[self.len + 1..self.len + 1 + frame.len()].copy_from_slice(frame);
        self.len += 1 + frame.len();
        self.frames += 1;
        true
    }

    /// Frames packed so far.
    pub fn frames(&self) -> usize {
        self.frames
    }

    /// True when no frame has been packed.
    pub fn is_empty(&self) -> bool {
        self.frames == 0
    }

    /// The datagram as built so far (header included).
    pub fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.len]
    }
}

fn valid_frame_len(n: usize) -> bool {
    (FRAME_HEADER_SIZE..=MAX_FRAME_SIZE).contains(&n)
}

/// What a received downlink datagram contains.
#[derive(Debug)]
pub enum Datagram<'a> {
    /// Legacy format: the datagram is exactly one frame.
    Single(&'a [u8]),
    /// A batch envelope: its header and an iterator over its frames.
    Batch(BatchHeader, Frames<'a>),
    /// A busy notice: the bridge belongs to the ground station at `owner`
    /// and dropped the recipient's uplink.
    Busy { owner: [u8; 4] },
    /// A batch marker with a version this build does not understand.
    Unsupported(u8),
    /// Empty, shorter than a frame header, or a truncated batch header.
    Malformed,
}

/// Classify a received downlink datagram.
pub fn classify(d: &[u8]) -> Datagram<'_> {
    match d.first() {
        None => Datagram::Malformed,
        Some(&MARKER) => {
            if d.len() < 2 {
                Datagram::Malformed
            } else if d[1] == KIND_BUSY {
                match d.get(2..BUSY_LEN) {
                    Some(&[a, b, c, e]) => Datagram::Busy { owner: [a, b, c, e] },
                    _ => Datagram::Malformed,
                }
            } else if d[1] != VERSION {
                Datagram::Unsupported(d[1])
            } else if d.len() < HEADER_LEN {
                Datagram::Malformed
            } else {
                Datagram::Batch(
                    BatchHeader::read(d),
                    Frames {
                        rest: &d[HEADER_LEN..],
                        truncated: false,
                    },
                )
            }
        }
        Some(_) if d.len() < FRAME_HEADER_SIZE => Datagram::Malformed,
        Some(_) => Datagram::Single(d),
    }
}

/// Iterator over the frames of a batch. Stops at the first entry whose
/// length is illegal or runs past the datagram, and sets [`Frames::truncated`];
/// frames before that point are still yielded.
#[derive(Clone, Debug)]
pub struct Frames<'a> {
    rest: &'a [u8],
    /// Set once iteration stopped on a bad or truncated entry.
    pub truncated: bool,
}

impl<'a> Iterator for Frames<'a> {
    type Item = &'a [u8];

    fn next(&mut self) -> Option<&'a [u8]> {
        let (&len, tail) = self.rest.split_first()?;
        let len = len as usize;
        if !valid_frame_len(len) || tail.len() < len {
            self.truncated = true;
            self.rest = &[];
            return None;
        }
        let (frame, rest) = tail.split_at(len);
        self.rest = rest;
        Some(frame)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const HDR: BatchHeader = BatchHeader {
        uart_errors: 1,
        dl_drops: 2,
        rtcm_rx: 0xBEEF,
        rtcm_drops: 4,
    };

    /// A max-size frame buffer with the given header; tests slice it.
    fn frame(id: u8, seq: u8) -> [u8; MAX_FRAME_SIZE] {
        let mut f = [0xAAu8; MAX_FRAME_SIZE];
        f[0] = id;
        f[1] = seq;
        f
    }

    #[test]
    fn round_trip_header_and_frames() {
        let mut buf = [0u8; MAX_DATAGRAM];
        let mut w = BatchWriter::new(&mut buf, HDR);
        let a = frame(26, 7);
        let b = frame(27, 8);
        assert!(w.try_push(&a[..62]));
        assert!(w.try_push(&b[..46]));
        assert_eq!(w.frames(), 2);
        let d = w.as_bytes();
        assert_eq!(d.len(), HEADER_LEN + 1 + 62 + 1 + 46);
        match classify(d) {
            Datagram::Batch(h, mut it) => {
                assert_eq!(h, HDR);
                assert_eq!(it.next(), Some(&a[..62]));
                assert_eq!(it.next(), Some(&b[..46]));
                assert_eq!(it.next(), None);
                assert!(!it.truncated);
            }
            other => panic!("expected batch, got {other:?}"),
        }
    }

    #[test]
    fn writer_respects_max_datagram_and_rejects_bad_frames() {
        let mut buf = [0u8; 4096];
        let mut w = BatchWriter::new(&mut buf, BatchHeader::default());
        let f = frame(18, 0);
        let mut n = 0;
        while w.try_push(&f) {
            n += 1;
        }
        // 1 + 130 bytes per entry after the 10-byte header.
        assert_eq!(n, (MAX_DATAGRAM - HEADER_LEN) / (1 + MAX_FRAME_SIZE));
        assert!(w.as_bytes().len() <= MAX_DATAGRAM);
        w.reset(BatchHeader::default());
        assert!(w.is_empty());
        assert!(!w.try_push(&f[..1]), "shorter than a frame header");
        let big = [1u8; MAX_FRAME_SIZE + 1];
        assert!(!w.try_push(&big), "longer than MAX_FRAME_SIZE");
        assert!(w.is_empty());
    }

    #[test]
    fn truncated_entry_stops_iteration_but_keeps_earlier_frames() {
        let mut buf = [0u8; 256];
        let mut w = BatchWriter::new(&mut buf, HDR);
        let a = frame(22, 1);
        assert!(w.try_push(&a[..10]));
        let mut d = w.as_bytes().to_vec_stack();
        d.push(50); // claims 50 bytes…
        d.extend(&[0u8; 10]); // …but only 10 follow
        match classify(d.as_slice()) {
            Datagram::Batch(_, mut it) => {
                assert_eq!(it.next(), Some(&a[..10]));
                assert_eq!(it.next(), None);
                assert!(it.truncated);
            }
            other => panic!("expected batch, got {other:?}"),
        }
    }

    #[test]
    fn illegal_entry_length_is_truncation() {
        let mut d = [0u8; HEADER_LEN + 3];
        HDR.write(&mut d);
        d[HEADER_LEN] = 1; // len 1 < FRAME_HEADER_SIZE
        match classify(&d) {
            Datagram::Batch(_, mut it) => {
                assert_eq!(it.next(), None);
                assert!(it.truncated);
            }
            other => panic!("expected batch, got {other:?}"),
        }
    }

    #[test]
    fn legacy_unsupported_and_malformed() {
        assert!(matches!(classify(&[]), Datagram::Malformed));
        assert!(matches!(classify(&[26]), Datagram::Malformed));
        assert!(matches!(classify(&[26, 0, 1, 2]), Datagram::Single(f) if f.len() == 4));
        assert!(matches!(classify(&[MARKER]), Datagram::Malformed));
        assert!(matches!(
            classify(&[MARKER, 9, 0, 0]),
            Datagram::Unsupported(9)
        ));
        assert!(matches!(
            classify(&[MARKER, VERSION, 0, 0]),
            Datagram::Malformed
        ));
        let mut empty = [0u8; HEADER_LEN];
        HDR.write(&mut empty);
        match classify(&empty) {
            Datagram::Batch(h, mut it) => {
                assert_eq!(h, HDR);
                assert_eq!(it.next(), None);
                assert!(!it.truncated);
            }
            other => panic!("expected empty batch, got {other:?}"),
        }
    }

    #[test]
    fn busy_notice_round_trip() {
        let n = busy_notice([192, 168, 50, 224]);
        assert_eq!(n.len(), BUSY_LEN);
        assert!(matches!(
            classify(&n),
            Datagram::Busy { owner: [192, 168, 50, 224] }
        ));
        assert!(matches!(classify(&n[..BUSY_LEN - 1]), Datagram::Malformed));
    }

    /// Tiny fixed-capacity Vec stand-in so the tests stay `no_std`.
    struct StackVec {
        buf: [u8; 512],
        len: usize,
    }
    impl StackVec {
        fn push(&mut self, b: u8) {
            self.buf[self.len] = b;
            self.len += 1;
        }
        fn extend(&mut self, s: &[u8]) {
            for &b in s {
                self.push(b);
            }
        }
        fn as_slice(&self) -> &[u8] {
            &self.buf[..self.len]
        }
    }
    trait ToStackVec {
        fn to_vec_stack(&self) -> StackVec;
    }
    impl ToStackVec for [u8] {
        fn to_vec_stack(&self) -> StackVec {
            let mut v = StackVec {
                buf: [0; 512],
                len: 0,
            };
            v.extend(self);
            v
        }
    }
}
