# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Summary

OCCTSwiftMesh is a Swift package providing **mesh-domain algorithms** for the OCCTSwift ecosystem. It operates on `OCCTSwift.Mesh` instances — extending what consumers can do with meshes produced by OCCT — without modifying OCCTSwift itself or extending OpenCASCADE.

**This package is deliberately not part of OCCTSwift.** OCCTSwift's mission is "Swift wrapper for OpenCASCADE Technology"; mesh decimation, smoothing, repair, remeshing, and similar are their own algorithm domain. OCCT's open-source distribution provides `BRepMesh_*` for mesh **generation** but no decimation or post-processing. OCCT-Components ships these as a paywalled module — this package fills the gap with permissive, vendored implementations.

```
Your App
  ├── OCCTSwift          (B-Rep solid modelling kernel — wraps OpenCASCADE)
  ├── OCCTSwiftMesh      (mesh-domain algorithms — this package)
  ├── OCCTSwiftViewport  (Metal viewport for rendering, no OCCT dep)
  └── OCCTSwiftScripts   (script harness + occtkit CLI verbs)
```

## Mission

Provide a single canonical implementation of mesh-domain algorithms that all OCCTSwift consumers (OCCTSwiftScripts, OCCTMCP, OCCTDesignLoop, UnfoldEngine, app code) can depend on. Avoid the failure mode of every consumer reimplementing OCCTSwift `Mesh` ↔ low-level-vertex-array marshalling locally.

## Knowledge Management

Durable, curated *what/why* knowledge for this repo lives in **`okf/`** — an OKF bundle (Open
Knowledge Format: categorized markdown + YAML frontmatter; see the ecosystem's
[`OKF-STANDARD.md`](https://github.com/SecondMouseAU/ecosystem/blob/main/OKF-STANDARD.md)).
Start at [`okf/index.md`](okf/index.md) for role, dependencies, and components; see
[`okf/policies/context-first.md`](okf/policies/context-first.md) (docs lookup order) and
[`okf/policies/docs-current.md`](okf/policies/docs-current.md) (docs must stay current) for the
mandatory policies. `CLAUDE.md` remains the detailed working reference for how to build, test,
and extend the code; `okf/` is the narrative for why it exists and how it relates to the rest of
the ecosystem.

## Current State

**Pre-alpha scaffold — no algorithms implemented yet.** The bridge has only a placeholder `OCCTMeshOptimizerABIVersion()` symbol; meshoptimizer is not yet vendored; `Mesh.simplified(_:)` does not exist. The full v0.1.0 implementation plan is in [`docs/INITIAL_IMPLEMENTATION.md`](docs/INITIAL_IMPLEMENTATION.md) — start there.

## Architecture

Three-layer wrapper, mirroring OCCTSwift's pattern:

```
Sources/OCCTSwiftMesh/      Public Swift API (extensions on Mesh, value types)
Sources/OCCTMeshOptimizer/  C++ bridge layer — vendors meshoptimizer
  include/                    C function declarations (single header)
  src/                        Bridge implementation + vendored .cpp files
Tests/OCCTSwiftMeshTests/   Swift Testing framework (@Suite / @Test)
```

The bridge target compiles meshoptimizer directly. No system dependencies; SPM `swift build` produces a fully self-contained library.

**Platform floor:** macOS 12+, iOS 15+. **C++ standard:** C++17 (`cxxLanguageStandard: .cxx17`). **`MESHOPTIMIZER_NO_EXPERIMENTAL`** is defined to `0` (experimental meshoptimizer APIs enabled) — this is intentional for the QEM-with-Hausdorff path. The SPM target uses `sources: ["src"]`, which compiles all `.cpp` files under `src/` recursively — meshoptimizer's own files are picked up automatically once vendored there.

### Vendored components

`Sources/OCCTMeshOptimizer/src/meshoptimizer/` holds the upstream meshoptimizer source tree, copied verbatim except for the addition of a top-level `LICENSE.md` file. **Do not modify vendored sources** — needed bug fixes go upstream first, then we re-vendor. Tag the vendored version in [NOTICE.md](NOTICE.md) and bump on re-vendor.

### Bridge naming

Mirror OCCTSwift's prefix conventions:
- C functions: `OCCTMeshSimplify`, `OCCTMeshSmoothLaplacian`, etc. — always start with `OCCTMesh`
- Opaque types: not needed in v0.1 (we operate on already-extracted vertex/index arrays passed by value)
- Result allocations: caller-owned, free via dedicated `OCCTMeshSimplificationResultRelease`

## Build & Test Commands

```bash
swift build                # Build the package
swift test                 # Run all tests
swift test --filter "..."  # Run a specific suite
```

The package depends on OCCTSwift via SPM (currently pinned to `from: "0.156.0"`). First build pulls OCCTSwift's xcframework; subsequent builds are incremental.

### Bumping the OCCTSwift dependency

When OCCTSwift ships new mesh-related surface we want to consume:

1. Bump `from: "..."` in `Package.swift` to the new minimum
2. Verify `swift build` is clean
3. Add tests covering the new surface
4. Update README.md "Status" + planned-API section if signatures change

Do not pin to a specific version — use `from:` to allow patch + minor bumps.

## Adding a New Mesh Operation

Pattern (follow exactly for each new algorithm):

1. **Spec in an issue first.** Mesh algorithms have many edge cases (boundary handling, attribute interpolation, topology preservation, numerical stability). Lock the API shape and acceptance criteria in an issue before writing code.
2. **Bridge declaration:** add C function in `Sources/OCCTMeshOptimizer/include/OCCTMeshOptimizer.h`
3. **Bridge implementation:** add C++ implementation in `Sources/OCCTMeshOptimizer/src/OCCTMeshOptimizerBridge.cpp` calling into vendored library
4. **Swift wrapper:** add extension method on `Mesh` (or relevant type) in `Sources/OCCTSwiftMesh/`. Public API surface uses Swift value types; never expose raw `OpaquePointer` to consumers
5. **Tests:** add `@Suite` in `Tests/OCCTSwiftMeshTests/` with at least: golden case, edge case, invalid-input rejection
6. **README:** add to "Planned API" → "API" once shipped
7. **CHANGELOG:** new entry with the operation, the algorithm, and the vendored-version reference if relevant

## Conventions

### Swift API style

- The package compiles under **Swift 6 strict concurrency** (`.swiftLanguageMode(.v6)`). All new types must satisfy the compiler's actor-isolation and `Sendable` requirements without suppressions unless genuinely needed.
- Mirror OCCTSwift's conventions where applicable: `Sendable` on value types; `@unchecked Sendable` on classes that wrap C handles; static factories preferred over throwing inits where the operation might fail (return optionals or `Result`-style structs).
- Options structs use `public var` properties so consumers can build them up via mutating assignment OR pass via `.init(...)`.
- Result structs are immutable (`public let`) once constructed.
- Operations that can fail return `Optional` or a typed `Result` struct with `isValid: Bool` — never `try` for "geometry didn't converge" failures (only for true exceptional conditions).

### Test framework

- Swift Testing (`@Suite`, `@Test`, `#expect`)
- **Never force-unwrap in `#expect`** — Swift Testing does NOT short-circuit. Use:
  ```swift
  if let r = result { #expect(r.something) }
  ```
- Tolerance-based assertions for floating point: `#expect(abs(a - b) < 1e-6)`

### Bridge style

- Wrap every algorithm call in `try { ... } catch (...) { return false; }` at the C ABI boundary — meshoptimizer doesn't throw, but defensive guards against future vendored libraries
- Out-parameter convention: pointers for primitives, `nullptr`-able where optional
- Memory ownership: caller-owned by default; results allocated by bridge are explicitly released via dedicated `*Release` functions

### Vendored library updates

- Pin the version in [NOTICE.md](NOTICE.md)
- Re-vendoring procedure documented in `docs/VENDORING.md` (write this when first vendoring)
- Test suite must pass against new vendored version before re-vendoring lands

## Release Process

Mirror OCCTSwift's flow:

1. Land all work for the release on `main` via PR(s)
2. Update README "Status" + version in `docs/CHANGELOG.md`
3. `swift build && swift test` — clean
4. `git tag v0.X.0 && git push origin v0.X.0`
5. `gh release create v0.X.0 --notes-file <release-notes.md>`

Tag patch releases (v0.X.Y) for tiny additive fixes; minor releases (v0.X.0) for new algorithms or substantive API additions.

## Relationship to Other Repos

| Repo | Role | Coupling |
|---|---|---|
| [OCCTSwift](https://github.com/gsdali/OCCTSwift) | Source of `Mesh` type | Hard SPM dependency |
| [OCCTSwiftViewport](https://github.com/gsdali/OCCTSwiftViewport) | Metal viewport | None — they don't talk |
| [OCCTSwiftScripts](https://github.com/gsdali/OCCTSwiftScripts) | `simplify-mesh` verb consumer | Will depend on us once #22 lands |
| [OCCTMCP](https://github.com/gsdali/OCCTMCP) | `simplify_mesh` tool consumer | Indirect via OCCTSwiftScripts |

**We don't modify OCCTSwift.** If we need new surface from OCCTSwift (e.g. a new vertex attribute getter), file an issue against OCCTSwift, get it released, then bump our `from:` pin.

## Documentation Standards

- **README.md** stays concise (~200 lines). Detailed content goes in `docs/`.
- **No stale plans or proposals** — delete docs when work is done or abandoned.
- **CHANGELOG.md** is the canonical release history.
- Per-issue / per-PR docs are ephemeral — don't commit them.

## What Goes Where

| Content | Location |
|---|---|
| Quick start, installation, planned API | `README.md` |
| How the bridge / vendoring works | `CLAUDE.md` (this file) |
| Vendoring procedure | `docs/VENDORING.md` |
| Release-by-release history | `docs/CHANGELOG.md` |
| Per-algorithm design notes | `docs/algorithms/<name>.md` (write per-algorithm) |
| License attributions | `NOTICE.md` |
| Curated what/why, ecosystem relationships, policies | `okf/index.md` |

## User Directives

- **Vendor permissively-licensed mesh algorithms** rather than expecting consumers to wire them up themselves.
- **Stay out of OCCTSwift's scope** — this package is a sibling, not an extension.
- **Public repo, LGPL-2.1** matching OCCTSwift.
- **Match OCCTSwift's release cadence and conventions** where it makes sense — same commit message style, CHANGELOG layout, README brevity.
