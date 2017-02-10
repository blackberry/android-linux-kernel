/* 
 *[FEATURE]-ADD by TCTSH.lijuan.wu, mini log, task-1488899
 * 
 */

#ifndef _SSRMINILOG_HEADER
#define _SSRMINILOG_HEADER


#ifdef CONFIG_TCT_SSR_MINILOG
extern void destroy_ssr_check_device(void);
extern int do_ssrminilog(void);
extern void ssr_check(const char*name, char * reason);
#else
static inline void destroy_ssr_check_device(void)
{
}

static inline int do_ssrminilog(void)
{
	return -ENODEV;
}

static inline void ssr_check(const char*name, char * reason)
{

}
#endif /* CONFIG_TCT_SSR_MINILOG */

#endif
