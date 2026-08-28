.PHONY: all catalog check clean FORCE

PDF_NAME := MFJA_Project_Handover_and_Maintenance_Manual.pdf

all: $(PDF_NAME)

catalog:
	python3 scripts/generate_catalog.py

FORCE:

main.pdf: FORCE main.tex preamble.tex metadata.tex $(wildcard chapters/*.tex) $(wildcard appendices/*.tex) scripts/generate_catalog.py
	python3 scripts/generate_catalog.py
	pdflatex -interaction=nonstopmode -halt-on-error main.tex
	pdflatex -interaction=nonstopmode -halt-on-error main.tex
	pdflatex -interaction=nonstopmode -halt-on-error main.tex

$(PDF_NAME): main.pdf
	cp main.pdf $(PDF_NAME)

check: $(PDF_NAME)
	python3 scripts/check_structure.py
	pdfinfo $(PDF_NAME) | grep -E '^(Pages|Page size|File size)'
	! grep -E 'undefined references|There were undefined|multiply defined' main.log
	! grep -E 'Overfull \\hbox|Overfull \\vbox' main.log
	! grep -R -E 'begin\{figure|includegraphics|begin\{tikzpicture|listoffigures' --include='*.tex' .
	grep -F '\listoftables' main.tex
	grep -F '\contentsline {table}' main.lot
	test "$$(grep -cF '\contentsline {table}' main.lot)" -eq "$$(grep -RhoF '\caption{' chapters appendices generated/repository_source_index.tex | wc -l)"
	! pdfimages -list $(PDF_NAME) | grep -E '^[[:space:]]*[0-9]+[[:space:]]+[0-9]+'
	! pdftotext $(PDF_NAME) - | grep -Ei 'snapshot_macros|docs/project_documentation/generated|docs/project_documentation/scripts/generate_catalog\.py|generated source catalogue|generated source catalog'

clean:
	rm -f main.aux main.log main.out main.toc main.lof main.lot main.pdf $(PDF_NAME)
	rm -f generated/repository_source_index.tex generated/repository_baseline.tex
