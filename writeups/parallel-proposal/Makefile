dummy: proposal.pdf

proposal.pdf: proposal.tex bib/proposal.bib
	xelatex --shell-escape proposal.tex
	biber proposal
	xelatex --shell-escape proposal.tex
