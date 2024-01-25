#!/bin/sh

C_FILES=$(find . -type f -name '*.[ch]' | grep -f ./style.c.include | grep -v -f ./style.c.exclude)
clang-format -i $C_FILES
