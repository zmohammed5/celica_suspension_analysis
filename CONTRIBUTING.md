# Contributing to Celica Suspension Analysis

Thank you for your interest in contributing! This document provides guidelines for contributing to the project.

## Code of Conduct

- Be respectful and inclusive
- Focus on constructive feedback
- Help others learn and grow

## How to Contribute

### Reporting Bugs

1. Check existing issues first
2. Use the bug report template
3. Include:
   - Clear description
   - Steps to reproduce
   - Expected vs actual behavior
   - System information
   - Relevant logs

### Suggesting Features

1. Check existing feature requests
2. Describe the use case
3. Explain the proposed solution
4. Consider implementation complexity

### Submitting Code

1. Fork the repository
2. Create a feature branch
   ```bash
   git checkout -b feature/your-feature-name
   ```
3. Make your changes
4. Write/update tests
5. Update documentation
6. Commit with clear messages
   ```bash
   git commit -m "Add feature: brief description"
   ```
7. Push and create PR

## Code Style

### Python
- Follow PEP 8
- Use type hints
- Write docstrings (Google style)
- Maximum line length: 100 characters

### JavaScript
- Use ES6+ features
- Consistent formatting (Prettier)
- Meaningful variable names

### Documentation
- Clear and concise
- Include examples
- Keep up to date

## Testing

- Write unit tests for new features
- Ensure existing tests pass
- Test on actual hardware when possible

```bash
# Run tests
pytest tests/

# Run with coverage
pytest --cov=src tests/
```

## Pull Request Process

1. Update README if needed
2. Update CHANGELOG
3. Ensure CI passes
4. Request review
5. Address feedback
6. Merge after approval

## Development Setup

```bash
# Clone your fork
git clone https://github.com/YOUR_USERNAME/celica_suspension_analysis.git
cd celica_suspension_analysis

# Create virtual environment
python -m venv venv
source venv/bin/activate

# Install dev dependencies
pip install -r requirements-dev.txt

# Run tests
pytest
```

## Questions?

- Open a discussion on GitHub
- Check existing documentation
- Ask in issues (tag as question)

Thank you for contributing!
