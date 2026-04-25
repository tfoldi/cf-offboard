# Agents

## Role

You are working on a single-process robotics control application in modern C++23.

## Design Constraints

- No ROS patterns
- No pub/sub abstractions
- No middleware layers

## State Management

- One authoritative state
- No duplicated state

## Control Model

- One fixed-rate loop
- Deterministic

## C++ Style Expectations

- Prefer pure functions
- Minimize shared mutable state
- Avoid hidden side effects
- Keep logic deterministic

## Design Sanity Check

- Can this be a pure function?
- Does this introduce hidden state?
- Does this resemble ROS?

If yes, redesign.
