
CFLAGS = -O3 -Wall -Wextra -Werror

TARGETS = extract
DATA_DIR = data_dir

all:	${TARGETS}

check:
	cppcheck -q *.c *.h

clean:
	rm -f ${TARGETS} ${DATA_DIR}/*.out

go:	${TARGETS}
	for i in ${DATA_DIR}/Disk* ; do ./extract $$i/*.raw >$$i.out; done
