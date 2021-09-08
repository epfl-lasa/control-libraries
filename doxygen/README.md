# Doxygen

This directory contains configurations for automatic documentation generation using doxygen.

Currently, only documentation for the [source](../source) folder is generated.

## Manual generation

Use `doxygen doxygen.conf` to generate an html directory in `docs/html`.

You can view the generated documentation by running `python -m http.server --directory docs/html`
and opening http://localhost:8000 in a browser.

## Automatic generation

A GitHub workflow automatically generates the html documentation and commits it
to the `docs` branch of this repo. GitHub Pages is configured to read the static
page content from this branch and serves it from the following URL:

https://epfl-lasa.github.io/control-libraries
