#ifndef SYSTEM_H
#define SYSTEM_H

#define IOWR(BASE,OFFSET,VALUE)  *((int *)((int)BASE+OFFSET))=VALUE
#define IORD(BASE, OFFSET)	*((int *)((int)BASE+OFFSET))

#endif