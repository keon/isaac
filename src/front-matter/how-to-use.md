# How to Use This Book

## Reader Archetypes and Fast Paths

This book serves different audiences with different goals. Choose your path:

### The Roboticist
You have a robot and need to simulate it. You care about physics accuracy and ROS integration.

**Fast path:** Chapters 1–4, 8–9, 11, 20–22

### The ML Engineer
You need to train policies or generate synthetic data. You care about throughput and transfer.

**Fast path for RL:** Chapters 1–3, 8–9, 28–31
**Fast path for synthetic data:** Chapters 1–2, 5–7, 25–27

### The Graphics Background
You know rendering and USD. You need robotics-specific patterns.

**Fast path:** Chapters 1–3, 8–11, 15–17

## Running Example: Mobile Manipulator in a Warehouse

Throughout this book, we build and refine a complete system:
- A mobile base with a 6-DOF arm
- RGB-D cameras and a 2D LiDAR
- ROS 2 navigation and manipulation stack
- Policy trained for bin picking
- Synthetic dataset for object detection

Each part adds a layer. By the end, you have a complete reference implementation.

## Repository Structure

All code examples are available at: `github.com/example/isaac-book-examples`

```
examples/
  part-01-mental-models/
  part-02-usd-scene/
  ...
  running-example/
    checkpoint-01-basic-robot/
    checkpoint-02-with-sensors/
    ...
```

Each chapter has runnable code. Each checkpoint is a working system.

## Time Estimates

- **Part I (Mental Models):** 2–3 hours reading, 1 hour experimenting
- **Part II (USD):** 3 hours reading, 2–3 hours building scenes
- **Part III (Robot Modeling):** 2 hours reading, 2–4 hours importing/tuning
- **Part IV (Sensors):** 3 hours reading, 1–2 hours configuration
- **Part V (Programming):** 4 hours reading, 3–5 hours scripting
- **Part VI (Integration):** 4 hours reading, 3–6 hours depending on stack
- **Part VII (Synthetic Data):** 2 hours reading, 4–8 hours dataset generation
- **Part VIII (Robot Learning):** 3 hours reading, many hours training
- **Part IX (End-to-End):** 3 hours reading, 4–8 hours integration
- **Part X (Production):** 3 hours reading, ongoing practice

**Total:** ~25–30 hours of focused reading, then project-specific practice.

## How Chapters Are Structured

Each chapter follows this pattern:

1. **What could go wrong** — the failure mode this chapter prevents
2. **Concepts** — mental models and system behavior
3. **Practical patterns** — how to implement correctly
4. **Diagnostic workflows** — how to debug when things break
5. **Running example** — applying concepts to the warehouse robot

Troubleshooting sections close each part, organized by symptoms.

## When to Read Linearly vs. Jump Around

**Read linearly if:**
- You're new to Isaac Sim
- You're building a system from scratch
- You want deep understanding

**Jump around if:**
- You have a specific problem (use Appendix B: Failure Mode Index)
- You're integrating an existing robot (use fast paths above)
- You're debugging (each part ends with troubleshooting)

## Prerequisites

This book assumes:
- Basic Python (you can read scripts and understand classes)
- Basic robotics (you know what a URDF is, roughly what ROS does)
- No USD knowledge required (we build it from first principles)
- No graphics/rendering knowledge required

If you have strong graphics background, you can skim Part II.
If you have Isaac Sim experience, start with Part III.

## Next Steps

If you're ready to start, proceed to [Part I: Mental Models](../part-1-mental-models/01-purpose-and-scope.md).

If you want to jump to a specific topic, see [Appendix E: Fast Paths](../appendices/e-fast-paths.md).
