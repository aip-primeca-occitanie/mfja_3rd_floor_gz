.PHONY: all check

all:
	pdflatex -interaction=nonstopmode -halt-on-error main.tex
	bibtex main
	pdflatex -interaction=nonstopmode -halt-on-error main.tex
	pdflatex -interaction=nonstopmode -halt-on-error main.tex

check: all
	pdfinfo main.pdf | grep -E '^(Pages|Page size|File size)'
	! grep -E 'undefined citations|undefined references|There were undefined' main.log
	! grep -E 'Overfull \\hbox|Overfull \\vbox' main.log
