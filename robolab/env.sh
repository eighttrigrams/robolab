#!/bin/bash

source env/bin/activate
alias g='python3 -m runner.start '
function e () {
	python3 -m "experimental.$1"
}
