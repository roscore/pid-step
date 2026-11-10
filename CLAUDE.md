# CLAUDE.md — pid-step (PUBLIC repository)

This repository is PUBLIC on GitHub. Treat every byte as world-readable forever.

## Hard privacy rules
- Never commit personal data: no real names, addresses, IPs, MAC addresses, GPS, tokens, emails
  (other than the noreply git identity), or host-specific absolute paths (home directories, NAS volumes).
- Examples must be synthetic and generic.
- The web app must work 100% client-side: no analytics, no trackers, no external CDNs/fonts,
  no network requests at runtime. User settings stay in the browser (optional localStorage only).
- No telemetry, no server component.

## Build
- Rust (stable) + wasm32-unknown-unknown + trunk.
- `cargo test` for the core crate must pass; `trunk build --release` must succeed.
- `sh tools/privacy_check.sh` must print "clean".
