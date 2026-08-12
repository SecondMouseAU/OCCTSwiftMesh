---
type: policy
title: Code style
description: Swift naming/API shape follows the Swift API Design Guidelines, formatting follows Google's Swift Style Guide via swift-format, and the OCCTMeshOptimizer C++ bridge follows OCCT's own clang-format; docs/ is the single source of truth for design rationale, not a second copy of it. Fully swept on rollout day, both Swift and the C++ bridge, no exemption manifest.
tags: [policy, style, swift, cpp, docs, agents]
timestamp: 2026-08-13
---

# Code style

**Naming and API shape** follow the
[Swift API Design Guidelines](https://www.swift.org/documentation/api-design-guidelines/) as-is:
clarity at the point of use over brevity, methods without side effects read as noun phrases,
methods with side effects as imperative verbs, boolean properties/methods read as assertions.

**Swift formatting and file layout** follow
[Google's Swift Style Guide](https://google.github.io/swift/), enforced by `swift-format`
(configured in `.swift-format`: 100-column limit, 4-space indent, the ecosystem's one deliberate
divergence from Google's own 2-space default, chosen to avoid a repo-wide reformat diff with no
readability gain). `swift-format lint --strict` is a blocking CI check
(`.github/workflows/code-style.yml`), whole-tree: formatting has no judgment call in it.

**SwiftLint is scoped to `orphaned_doc_comment` only** (`.swiftlint.yml`, `only_rules`, not the
default set). SwiftLint's defaults duplicate `swift-format`'s formatting opinions (and can disagree
with them on the same line) and separately add a large code-quality/complexity surface
(`identifier_name`, `cyclomatic_complexity`, `function_body_length`, `nesting`, ...) that overlaps
the ecosystem's own [code-structure](code-structure.md) policy rather than this one; a file that
needs a structural pass runs one as its own scoped initiative, not as a side effect of a
style-lint gate. `orphaned_doc_comment` is the one rule left that catches something `swift-format`
has no equivalent for: a doc comment separated from its declaration.

**The `OCCTMeshOptimizer` C++ bridge** follows OCCT's own `.clang-format`, checked in at
`Sources/OCCTMeshOptimizer/.clang-format` (this repo has no local `Libraries/occt-src` to copy
from directly, since it depends on OCCTSwift as a Swift package rather than vendoring OCCT source
itself; the checked-in copy is instead sourced from OCCTSwift's own `Sources/OCCTBridge/.clang-format`,
which is itself a checked-in copy of OCCT's vendored config). Scope is deliberately narrow:
`Sources/OCCTMeshOptimizer` holds both a small first-party bridge (`include/OCCTMeshOptimizer.h`,
`src/OCCTMeshOptimizerBridge.cpp`, roughly 200 lines total) and a vendored third-party library
(`src/meshoptimizer/*`, 19 `.cpp` files plus `meshoptimizer.h`, MIT-licensed, from
[zeux/meshoptimizer](https://github.com/zeux/meshoptimizer) v1.1). Only the first-party bridge is
in scope for `clang-format`: `docs/VENDORING.md` states outright "Don't modify vendored sources.
Bug fixes go upstream first," and reformatting the vendored subtree would violate that directly,
the same treatment vendored OCCT source itself gets elsewhere in the ecosystem. A second
`.clang-format` at `Sources/OCCTMeshOptimizer/src/meshoptimizer/.clang-format` sets
`DisableFormat: true` so an editor or IDE walking up the directory tree does not pick up the OCCT
style for the vendored files either.

**Doc comments stay terse.** A `///` comment is a single-sentence summary plus only the
`Parameter`/`Returns`/`Throws` tags that add something the summary doesn't already say. Design
rationale, extended examples, and issue cross-references belong in `docs/`, not duplicated in
source: `docs/` is the single source of truth for *why* and *how*, per
[GitLab's documentation style guide](https://docs.gitlab.com/development/documentation/styleguide/)
("share the link to the documentation instead of rephrasing the information").
`Scripts/comment-ratio-check.sh` flags (never fails) a file whose comment lines outnumber its code
lines, as a signal for review, not an automatic failure.

## Full sweep, not a gradual manifest

Unlike OCCTSwift (large, gradual exemption-manifest rollout) and matching OCCTSwiftScripts (the
ecosystem's pilot repo), this repo was small enough to sweep into full compliance in one PR on
both halves at once:

- **Swift**: ~6,600 lines across ~43 files (`Sources/` + `Tests/`). `swift-format
  format --in-place --recursive` ran across the whole tree, and every remaining
  `swift-format lint --strict` / `swiftlint lint --strict` finding (doc-comment structure,
  missing `Returns:` sections, a handful of single-letter variable names, over-length end-of-line
  comments) was hand-fixed to zero, not grandfathered.
- **C++ bridge**: measured first, not assumed. A real `clang-format --dry-run --Werror -style=file`
  run against the two first-party bridge files showed a small, fully mechanical diff (reindentation,
  parameter-list realignment, no semantic change), so the bridge was swept too rather than put on
  an exemption list.

No `Scripts/check-style-manifest.py`-style mechanism exists here: every file complies from this
PR forward, so there is nothing left to grandfather. New code complies from creation, the same as
everywhere else in the ecosystem.

Why: the ecosystem-wide proposal and evidence (comment:code ratios, a live doc-drift bug found in
`OCCTSwift`'s `docs/reference/CurveAdaptors.md`) live in
[`ecosystem` docs/code-style-policy-proposal-2026-08.md](https://github.com/SecondMouseAU/ecosystem/blob/main/docs/code-style-policy-proposal-2026-08.md).
Precedent: [OCCTSwiftScripts#114](https://github.com/SecondMouseAU/OCCTSwiftScripts/issues/114) /
[#115](https://github.com/SecondMouseAU/OCCTSwiftScripts/pull/115) (full sweep, small pure-Swift
repo) and [OCCTSwift#876](https://github.com/SecondMouseAU/OCCTSwift/issues/876) /
[#878](https://github.com/SecondMouseAU/OCCTSwift/pull/878) (gradual manifest, large repo with a
real bridge layer).

Ecosystem standard: see
[OKF-STANDARD.md](https://github.com/SecondMouseAU/ecosystem/blob/main/OKF-STANDARD.md).
