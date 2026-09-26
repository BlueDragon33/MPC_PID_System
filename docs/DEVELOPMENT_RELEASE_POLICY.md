# Standalone Development / Release Policy

This repository follows the shared BlueDragon33 development policy.

## Default now: Standalone Development Mode

- Each application is an independent web-app.
- Local-first is the default: the UI opens without waiting for the central manager, remote device approval, or a live control plane.
- Internet is optional for access rights, synchronization, remote contracts, cloud data, and publishing.
- Missing manager/contract/approval/preview credentials must not block the app shell.
- `main` is the development-live branch where a live host exists; pushes should publish automatically with a lightweight workflow.
- Heavy release validation remains manual during active development.
- Stale publish runs should be cancelled in favor of the newest `main` revision.
- Application Management is a convenience/control center, not a runtime dependency for other apps.

## Application Management access switch

The central access/approval gate is OFF by default in Standalone Development Mode. Turning it ON re-enables online rights/device verification for remote mutations.

## Release Mode

On explicit release request: freeze revision, switch access policy to `managed` where supported, run full release gates, verify migrations/artifacts/preview, deploy protected Production, run exact-revision smoke/read-back, record version, then return to Standalone Development Mode unless requested otherwise.

Do not silently restore heavy gates during ordinary feature work. Keep release safeguards available for explicit Release Mode.