# Introduction

Welcome to **Isaac: A Systems-Oriented Guide to Isaac Sim**.

This book is a comprehensive, engineering-focused guide to NVIDIA Isaac Sim for robotics simulation, synthetic data generation, and robot learning.

## What is this book?

This is not a tutorial collection or API reference. This is a **systems book** that teaches you:
- How Isaac Sim actually works (execution models, guarantees, limitations)
- How to build production-grade simulation pipelines
- How to diagnose and fix problems systematically
- How to scale from prototype to deployment

## Who is this book for?

This book serves three primary audiences:

- **Roboticists** integrating Isaac Sim with existing robot stacks (especially ROS 2)
- **ML Engineers** training policies or generating synthetic data
- **Graphics/Simulation Engineers** coming from USD/Omniverse backgrounds

Each reader archetype has a recommended fast path through the material.

## How to use this book

**If you're new to Isaac Sim:** Start with [Mental Models](./mental-models/01-purpose-and-scope.md) to build foundational understanding.

**If you have a specific problem:** Jump to [Failure Mode Index](./appendices/b-failure-mode-index.md) for symptom-based diagnosis.

**If you want to understand deeply:** Read linearly (~25-30 hours of focused study).

**For goal-oriented reading:** See [Fast Paths](./appendices/e-fast-paths.md) for curated paths by use case.

## What makes this book different?

- **Systems perspective**: Focuses on execution models and contracts, not just APIs
- **Diagnostic workflows**: Each section ends with troubleshooting guides
- **Production-oriented**: Covers reliability, performance, and sim-to-real transfer
- **Running example**: A mobile manipulator in a warehouse, built incrementally

## Quick Start

Begin with [Mental Models](./mental-models/01-purpose-and-scope.md) to understand how Isaac Sim works before building with it.
