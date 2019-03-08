/*
  utils.h - Utility functions
  Created by Cyrill Lippuner, 2019.
*/
#ifndef Utils_h
#define Utils_h

/**
 * Sort compare criteria (DESC)
 * qsort requires you to create a sort function
 */
int sort_desc(const void *cmp1, const void *cmp2);

/**
* Sort compare criteria (DESC)
 * qsort requires you to create a sort function
 */
int sort_asc(const void *cmp1, const void *cmp2);

#endif
