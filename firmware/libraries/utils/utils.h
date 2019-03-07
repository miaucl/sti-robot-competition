/*
  utils.h - Utility functions
  Created by Cyrill Lippuner, 2019.
*/
#ifndef Utils_h
#define Utils_h

/* Sort functions */

// qsort requires you to create a sort function
int sort_desc(const void *cmp1, const void *cmp2)
{
  // Need to cast the void * to int *
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);
  // The comparison
  return a > b ? -1 : (a < b ? 1 : 0);
  // A simpler, probably faster way:
  //return b - a;
}
int sort_asc(const void *cmp1, const void *cmp2)
{
  // Need to cast the void * to int *
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);
  // The comparison
  return a < b ? -1 : (a > b ? 1 : 0);
  // A simpler, probably faster way:
  //return b - a;
}


#endif
