# Releasing cybflight-msgs

The `Publish crate` workflow publishes a stable GitHub release tagged `v<package version>` from `cybird-robotics/cybflight-msgs`. Pushes and pull requests only run checks. Publish this dependency before updating applications that require the new version.

Before the first release, add a crates.io publishing token as a repository Actions secret named `CRATES_IO_TOKEN` (Settings → Secrets and variables → Actions → New repository secret). The workflow maps this secret to Cargo's `CARGO_REGISTRY_TOKEN` environment variable. No GitHub environment setup is required. Keep crate ownership with the maintainers responsible for this package.

1. Update the crate version and its changelog/release notes.
2. Run the CI checks and `cargo publish -p cybflight-msgs --dry-run --locked` in a clean checkout.
3. Review the package file list with `cargo package -p cybflight-msgs --list`.
4. Commit and tag the reviewed revision, then publish its GitHub release.
5. Verify the version on crates.io before changing consumer lockfiles.

The workflow checks that the tag matches the manifest and publishes only this crate.
