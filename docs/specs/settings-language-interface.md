# Settings Language and Interface Specification

## Goal

Make the MPC-PID workbench readable at ChatGPT-like text sizing, visually balance every navigation/action icon, and give users persistent language and interface controls without changing any control-system algorithms.

## Language

- Supported modes: English, Russian, Vietnamese, English–Russian, Vietnamese–Russian, and English–Vietnamese.
- A monolingual mode renders one localized label.
- A bilingual mode renders the primary and secondary translations separated by ` · `.
- Control acronyms, symbols, formulas, solver names, and repository identifiers remain unchanged.
- Language changes apply immediately and persist on the current device.

## Interface

- Default body text is 16 px; selectable sizes are 14, 16, and 18 px.
- Background choices are Dark, Light, and Soft gray.
- Font choices are Inter, System, and Serif.
- Contrast choices are Standard and High.
- All preferences apply immediately, persist locally, and recover safely from invalid stored values.
- Existing theme control remains functional and synchronized with the background preference.

## Icons and layout

- Use the existing Lucide icon family only.
- Navigation, button, heading, and status icons use consistent optical sizes and square alignment boxes.
- Icons never shrink, distort, or displace labels on narrow screens.
- Settings remain responsive and keyboard accessible.

## Constraints

- Do not alter simulation, controller, estimator, solver, experiment, or safety logic.
- Do not add a network dependency.
- Keep GitHub Pages static-deployment compatibility.

