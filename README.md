# TurtleTracerDocs Documentation Workspace

This repository serves as the **central documentation and reference** project for the
Turtle Tracer ecosystem. It contains utility docs, build scripts, and two
related sub‑projects used by the robotics community:

- **TurtleTracerLib** – an Android/Gradle project implementing pathfinding
  algorithms for FTC robotics.
- **TurtleTracer** – a Svelte/Vite based visualization tool that
  demonstrates and tests pathfinding behavior.

---

Each subproject is self‑contained but shares reference data updated by the
`npm run ref` script at the root.

### Key Subdirectories

| Folder                 | Purpose                                               |
| ---------------------- | ----------------------------------------------------- |
| `TurtleTracerLib/app/` | Android app modules, source, tests, and Gradle config |
| `TurtleTracer/src/`    | Svelte components, utilities, and plugin examples     |
| `public/`              | Public assets used by the documentation site          |

---

## Getting Started

1. **Clone the repository** (if not already):

   ```bash
   git clone https://github.com/Mallen220/TurtleTracerDocs.git
   cd TurtleTracerDocs
   ```

2. **Install dependencies**:

   ```bash
   npm install          # installs tools for docs, visualizer, etc.
   ```

3. **Update cross‑references**:
   ```bash
   npm run ref
   ```
   This script scans the workspace and updates any outdated links or imports.

---

## Documentation Site

This workspace uses [Astro](https://docs.astro.build) with the Starlight
template for hosting documentation. Content is stored under
`src/content/docs/`.

Commands available from the root:

| Command              | Description                                |
| -------------------- | ------------------------------------------ |
| `npm run dev`        | Start local docs server (`localhost:4321`) |
| `npm run build`      | Build static site to `./dist/`             |
| `npm run preview`    | Preview the generated site                 |
| `npm run astro -- …` | Run Astro CLI commands                     |

Add markdown files under `src/content/docs/` to extend project documentation.

---

## License

This project is open for everyone\*! We believe in the power of open source and community collaboration. You are free to use, modify, and distribute this software as you see fit. All we ask is that you give credit to the original developers and any contributors who have helped shape this tool. If you make improvements, please consider sharing them back with the community! Please note this project uses a modified version of the Apache 2.0 license.

See the [LICENSE](LICENSE) file for the full modified Apache 2.0 legal text.

---
