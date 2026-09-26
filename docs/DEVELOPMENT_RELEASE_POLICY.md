# Development / Release Mode Policy

This repository follows the shared BlueDragon33 development policy.

## Default mode: Development Mode

During active development, optimize for iteration speed.

- Use lightweight Fast CI only.
- Avoid running full browser suites, full-system integration, large migration audits, research benchmarks, or release-readiness gates on every change.
- Preview/publish paths should be automatic or best-effort where infrastructure credentials exist.
- Missing preview credentials must not block ordinary development work; skip the optional preview cleanly and report why.
- Cancel stale preview runs when a newer revision supersedes them.
- Do not mutate Production automatically unless this repository already intentionally uses a simple static production publish flow.
- Keep heavy validation workflows available as manual workflows rather than deleting their purpose from the project.
- Do not interpret a skipped optional preview as a successful live deployment.

## Release Mode

Release Mode starts only when the owner explicitly asks to prepare/release/publish a stable production version.

Before releasing:

1. Freeze the intended release revision.
2. Restore or manually run the repository's heavy validation gates as appropriate.
3. Run required lint, typecheck, unit/contract, browser/E2E, integration, schema/migration, artifact and security/boundary checks relevant to this app.
4. Resolve all release-blocking failures.
5. Build and verify the exact release artifact/revision.
6. Verify Preview when the app has a Preview environment.
7. Deploy Production using the repository's protected production path.
8. Perform post-deploy read-back/smoke checks on the exact deployed revision.
9. Record the release version/commit and return the repository to Development Mode unless the owner requests otherwise.

## Important

Development Mode intentionally trades exhaustive validation for iteration speed. Release Mode restores the stricter safety gates.

Do not silently switch to Release Mode during ordinary feature work. Do not silently weaken Release Mode once a production release has been requested.
