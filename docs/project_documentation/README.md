# MFJA Project Handover and Maintenance Manual

## Purpose

This directory contains the English LaTeX source and release PDF for the MFJA
third-floor ROS 2 and Gazebo engineering handover manual. The manual documents
project architecture, runtime data flow, package ownership, operation, testing,
maintenance, safety boundaries, and source-file locations.

The release is text-and-table only. Every substantive table has a caption,
label, and visible row separators, and the PDF includes a populated List of
Tables. Technical claims identify their governing repository paths through a
source table, source column, or consolidated source note.

## Directory Layout

- `main.tex` defines document order and includes the front matter, chapters,
  and appendices.
- `chapters/` contains the maintained narrative and operational guidance.
- `appendices/` contains the glossary, interface reference, launch reference,
  and repository source index wrappers.
- `generated/` contains build-produced LaTeX inputs; do not edit them by hand.
- `scripts/generate_catalog.py` builds the repository source index and snapshot
  metadata.
- `scripts/check_structure.py` validates document structure and presentation.
- `MFJA_Project_Handover_and_Maintenance_Manual.pdf` is the named release PDF.

## Requirements

Building requires Python 3 and a TeX Live installation that provides
`pdflatex`, `tcolorbox`, `longtable`, and the TeX Gyre fonts. Validation also
uses `pdfinfo`, `pdftotext`, and `pdfimages` from Ubuntu's `poppler-utils`
package.

## Build

Run the following command from `docs/project_documentation/`:

```bash
make
```

The build refreshes the repository source index, runs LaTeX three times so
references and table lists settle, and writes:

```text
MFJA_Project_Handover_and_Maintenance_Manual.pdf
```

To refresh only the repository source index and snapshot metadata:

```bash
make catalog
```

## Validation

Run the complete validation target before releasing the manual:

```bash
make check
```

The validation checks:

- PDF creation and A4 metadata;
- undefined or multiply defined references;
- overfull boxes;
- absence of figure, image, TikZ, and List of Figures markup;
- absence of embedded raster images in the compiled PDF;
- absence of build-internal paths or catalogue wording in reader-facing text;
- presence of the List of Tables and its entries;
- unique captions and labels for substantive tables; and
- visible separators between successive table rows.

After the automated checks, inspect the title page, Contents, List of Tables,
representative long tables, wrapped source paths, and the final appendix pages.

## Generated-File Policy

The source index is reproducible only for the same branch and commit. Its
generator refuses staged, unstaged, or untracked content inside the four ROS
package directories so mutable package content cannot be described as the
documented repository revision. Untracked files elsewhere are not included in
the index.

Do not edit files under `generated/` directly. Change the index builder or the
underlying repository source, commit the intended package state, and rebuild.

## Release Procedure

1. Confirm the intended branch, commit, and clean state of all four ROS package
   directories.
2. Run `make check` and review the resulting PDF visually.
3. Review changes to the maintained LaTeX sources and repository source index.
4. Confirm that the named PDF matches the reviewed source revision.
5. Add this complete directory, including the release PDF, to version control
   or a controlled release archive.

An untracked local documentation directory will not be present in a fresh
clone, so archive or commit it before handover.
