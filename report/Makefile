.PHONY: all check

all:
	pdflatex -interaction=nonstopmode -halt-on-error main.tex
	bibtex main
	pdflatex -interaction=nonstopmode -halt-on-error main.tex
	pdflatex -interaction=nonstopmode -halt-on-error main.tex
	cp main.pdf IBRAHIM_Ali_Master_ISC_FI_Parcours_RI_RAPPORT_FINAL_le18Aout2026.pdf

check: all
	pdfinfo main.pdf | grep -E '^(Pages|Page size|File size)'
	! grep -E 'undefined citations|undefined references|There were undefined' main.log
	! grep -E 'Overfull \\hbox|Overfull \\vbox' main.log
