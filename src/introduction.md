# Introduction

Welcome to **Isaac: A Systems-Oriented Guide to Isaac Sim**.

This book is a comprehensive, engineering-focused guide to NVIDIA Isaac Sim for robotics simulation, synthetic data generation, and robot learning.

## What is this book?

This is not a tutorial collection or API reference. This is a **systems book** that teaches you:
- How Isaac Sim actually works (execution models, guarantees, limitations)
- How to diagnose problems systematically
- How to build production-grade simulation pipelines
- How to avoid common failure modes that waste days

## Who is this book for?

This book serves three primary audiences:

- **Roboticists** integrating Isaac Sim with existing robot stacks (especially ROS 2)
- **ML Engineers** training policies or generating synthetic data
- **Graphics/Simulation Engineers** coming from USD/Omniverse backgrounds

Each reader archetype has a recommended fast path through the material.

## How to use this book

**If you're new to Isaac Sim:** Start with [Part I: Mental Models](./mental-models/01-purpose-and-scope.md) to build foundational understanding.

**If you have a specific problem:** Jump to [Appendix B: Failure Mode Index](./appendices/b-failure-mode-index.md) for symptom-based diagnosis.

**If you want to understand deeply:** Read Parts I–X linearly (~25-30 hours of focused study).

**For goal-oriented reading:** See [Appendix E: Fast Paths](./appendices/e-fast-paths.md) for curated paths by use case.

## What makes this book different?

- **Failure-mode driven**: Each chapter starts with "what could go wrong"
- **Systems perspective**: Focuses on execution models and contracts, not just APIs
- **Troubleshooting integrated**: Each part ends with diagnostic workflows
- **Production-oriented**: Covers reliability, performance, and sim-to-real transfer
- **Running example**: A mobile manipulator in a warehouse, built incrementally

## Quick Start

Begin with [Part I: Mental Models](./mental-models/01-purpose-and-scope.md) to build the foundational understanding that prevents early mistakes.
