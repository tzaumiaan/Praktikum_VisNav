OUT = visnav_ex2_report
#PMB = preamble

TEX = pdflatex -shell-escape -interaction=nonstopmode -file-line-error
#PRE =  $(TEX) -ini -job-name="$(PMB)" "&pdflatex $(PMB).tex\dump"
BIB = bibtex

.PHONY: all view

all : $(OUT).aux $(OUT).pdf
# first generate aux for cross reference
# then redo all procedure to generate pdf

view :
	open $(OUT).pdf

check : $(OUT).tex
	aspell -c -t $(OUT).tex -d en

$(OUT).pdf : $(OUT).tex $(OUT).bbl $(OUT).blg
	$(TEX) $(OUT).tex

$(OUT).bbl $(OUT).blg : $(OUT).bib $(OUT).aux
	$(BIB) $(OUT)

$(OUT).aux : $(OUT).tex
	$(TEX) $(OUT).tex

$(OUT).bib : $(OUT).tex
	$(TEX) $(OUT).tex

#$(PMB).fmt : $(PMB).tex
#	$(PRE) $(PMB).tex

clean :
	rm -rf *.bbl *.blg *.aux *.bib *.fmt $(OUT).pdf *.log *.bak
