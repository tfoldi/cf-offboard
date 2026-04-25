# C++ Guidelines

## Core Principles

### Value-oriented design
Use structs and explicit data.

### Functional style
Prefer pure functions.

### Minimize mutation
State changes only at boundaries.

### Explicit data flow
No hidden dependencies.

### Composition over inheritance

### Strong typing

### Error handling
Use std::expected.

### Side effects at edges only

### Determinism first

### Keep concurrency simple

## Anti-patterns

- callback graphs
- pub/sub inside process
- hidden state

## Practical Rule

If it’s not testable or replayable, rethink.
