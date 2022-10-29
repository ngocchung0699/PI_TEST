#ifndef _KERNEL
#include <sys/cdefs.h>
#endif

#define O_SYNC 2

/*
 * Protections are chosen from these bits, or-ed together
 */
#define	PROT_NONE	0x00	/* no permissions */
#define	PROT_READ	0x01	/* pages can be read */
#define	PROT_WRITE	0x02	/* pages can be written */
#define	PROT_EXEC	0x04	/* pages can be executed */

/*
 * Flags contain sharing type and options.
 * Sharing types; choose one.
 */
#define	MAP_SHARED	0x0001	/* share changes */
#define	MAP_PRIVATE	0x0002	/* changes are private */

/*
 * Other flags
 */
#define	MAP_FIXED	0x0010	/* map addr must be exactly as requested */
#define	__MAP_NOREPLACE	0x0800	/* fail if address not available */
#define	MAP_ANON	0x1000	/* allocated from memory, swap space */
#define	MAP_ANONYMOUS	MAP_ANON	/* alternate POSIX spelling */
#define	__MAP_NOFAULT	0x2000
#define	MAP_STACK	0x4000	/* mapping is used for a stack */
#define	MAP_CONCEAL	0x8000	/* omit from dumps */

#define	MAP_FLAGMASK	0xfff7

#ifndef _KERNEL
/*
 * Legacy defines for userland source compatibility.
 * Can be removed once no longer needed in base and ports.
 */
#define	MAP_COPY		MAP_PRIVATE	/* "copy" region at mmap time */
#define	MAP_FILE		0	/* map from file (default) */
#define	MAP_HASSEMAPHORE	0	/* region may contain semaphores */
#define	MAP_INHERIT		0	/* region is retained after exec */
#define	MAP_NOEXTEND		0	/* for MAP_FILE, don't change file size */
#define	MAP_NORESERVE		0	/* Sun: don't reserve needed swap area */
#define	MAP_RENAME		0	/* Sun: rename private pages to file */
#define	MAP_TRYFIXED		0	/* attempt hint address, even within heap */
#endif

/*
 * Error return from mmap()
 */
#define MAP_FAILED	((void *)-1)

/*
 * POSIX memory advisory values.
 * Note: keep consistent with the original definitions below.
 */
#define	POSIX_MADV_NORMAL	0	/* no further special treatment */
#define	POSIX_MADV_RANDOM	1	/* expect random page references */
#define	POSIX_MADV_SEQUENTIAL	2	/* expect sequential page references */
#define	POSIX_MADV_WILLNEED	3	/* will need these pages */
#define	POSIX_MADV_DONTNEED	4	/* don't need these pages */

#if __BSD_VISIBLE
/*
 * Original advice values, equivalent to POSIX definitions,
 * and few implementation-specific ones.  For in-kernel and historic use.
 */
#define	MADV_NORMAL		POSIX_MADV_NORMAL
#define	MADV_RANDOM		POSIX_MADV_RANDOM
#define	MADV_SEQUENTIAL		POSIX_MADV_SEQUENTIAL
#define	MADV_WILLNEED		POSIX_MADV_WILLNEED
#define	MADV_DONTNEED		POSIX_MADV_DONTNEED
#define	MADV_SPACEAVAIL		5	/* insure that resources are reserved */
#define	MADV_FREE		6	/* pages are empty, free them */
#endif

/*
 * Flags to minherit
 */
#define MAP_INHERIT_SHARE	0	/* share with child */
#define MAP_INHERIT_COPY	1	/* copy into child */
#define MAP_INHERIT_NONE	2	/* absent from child */
#define MAP_INHERIT_ZERO	3	/* zero in child */

/*
 * Flags to msync
 */
#define	MS_ASYNC	0x01	/* perform asynchronous writes */
#define	MS_SYNC		0x02	/* perform synchronous writes */
#define	MS_INVALIDATE	0x04	/* invalidate cached data */

/*
 * Flags to mlockall
 */
#define	MCL_CURRENT	0x01	/* lock all pages currently mapped */
#define	MCL_FUTURE	0x02	/* lock all pages mapped in the future */

#ifndef _KERNEL
//#include <sys/_types.h>



#ifndef _SIZE_T_DEFINED_
#define _SIZE_T_DEFINED_
typedef struct int	size_t;
#endif

#ifndef _OFF_T_DEFINED_
#define _OFF_T_DEFINED_
typedef struct int		off_t;
#endif

__BEGIN_DECLS
void *	mmap(void *, size_t, int, int, int, off_t);
int	mprotect(void *, size_t, int);
int	munmap(void *, size_t);
int	msync(void *, size_t, int);
int	mlock(const void *, size_t);
int	munlock(const void *, size_t);
int	mlockall(int);
int	munlockall(void);
#if __BSD_VISIBLE
int	madvise(void *, size_t, int);
int	minherit(void *, size_t, int);
int	mimmutable(void *, size_t);
void *	mquery(void *, size_t, int, int, int, off_t);
#endif
 __mode_t;
int	posix_madvise(void *, size_t, int);
int	shm_open(const char *, int, __mode_t);
int	shm_unlink(const char *);
int	shm_mkstemp(char *);
__END_DECLS

#endif /* !_KERNEL */