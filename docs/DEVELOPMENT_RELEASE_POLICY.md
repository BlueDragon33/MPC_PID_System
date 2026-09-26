# Shared Development / Release Policy

## Default now: Development Live Mode

This repository is in the shared BlueDragon33 Development Live Mode.

- `main` is the live development source when a live host is configured.
- Use the dedicated lightweight **Live Development Deploy** path; do not require Preview promotion.
- Access defaults to **standalone/local-first**.
- Application Management is optional coordination/launch/sync infrastructure, not a mandatory gate for opening the app.
- Device approval, remote-admin authorization and central control are optional during active development.
- The app must remain usable as a local web-app whenever its own runtime supports local execution.
- Internet should mainly be needed for publishing, optional access-right lookup, synchronization and remote features.
- Missing central-management connectivity must not prevent the core app from opening in standalone development mode.
- Heavy CI, browser suites, integration gates and protected Production promotion remain manual during active development.
- Never treat a skipped optional sync/management bridge as a failed local app.

## Application Management access switch

When this app participates in Application Management:

- **Approval/Managed ON**: use device/access approval and central management.
- **Approval/Managed OFF**: enter directly in standalone local-first mode.
- Development defaults to OFF unless explicitly switched on by the user.

## Release Mode

Release Mode starts only when the owner explicitly requests a stable/production release.

At release time:

1. Freeze the release revision.
2. Switch access mode back to managed/secure where required.
3. Restore or manually run the heavy release gates.
4. Validate database/schema migrations and remote contracts that are part of the release.
5. Verify the exact release artifact.
6. Deploy using the protected Release/Production workflow.
7. Run post-deploy smoke/read-back checks.
8. Record the release revision/version.
9. Return to Development Live Mode unless instructed otherwise.

The Release workflow must remain available even while Development Live Mode is the default.
