# Isaac

A comprehensive book/documentation project built with mdBook.

## 📚 Read the Book

The book is available online at: `https://your-username.github.io/isaac/`

## 🚀 Local Development

### Prerequisites

Install mdBook:

```bash
# macOS
brew install mdbook

# Linux
cargo install mdbook

# Or download from https://github.com/rust-lang/mdBook/releases
```

### Building the Book

```bash
# Build the book
mdbook build

# Serve locally with live reload
mdbook serve
```

The book will be available at `http://localhost:3000`

## 📁 Project Structure

```
.
├── book.toml           # mdBook configuration
├── src/                # Book source files (Markdown)
│   ├── SUMMARY.md      # Table of contents
│   ├── introduction.md
│   ├── mental-models/
│   ├── usd-scene/
│   ├── robot-modeling/
│   ├── sensors/
│   ├── programming/
│   ├── integration/
│   ├── synthetic-data/
│   ├── robot-learning/
│   ├── end-to-end/
│   ├── production/
│   └── appendices/
└── docs/               # Generated HTML (published to GitHub Pages)
```

## 🔧 Writing Content

1. Edit Markdown files in the `src/` directory
2. Update `src/SUMMARY.md` to modify the table of contents
3. Run `mdbook serve` to preview changes locally
4. Commit and push to deploy automatically via GitHub Actions

## 🌐 GitHub Pages Setup

To enable GitHub Pages:

1. Go to your repository settings
2. Navigate to **Pages** under "Code and automation"
3. Under **Source**, select "GitHub Actions"
4. Push to main branch to trigger deployment

## 📝 Configuration

Edit `book.toml` to customize:
- Book title and authors
- Repository URL
- Output directory
- Theme and styling options

See [mdBook documentation](https://rust-lang.github.io/mdBook/) for more options.

## 📖 mdBook Resources

- [mdBook Documentation](https://rust-lang.github.io/mdBook/)
- [mdBook GitHub Repository](https://github.com/rust-lang/mdBook)
- [Markdown Guide](https://www.markdownguide.org/)

## 📄 License

[Add your license here]
