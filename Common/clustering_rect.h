#ifndef CLUSTERING_RECT_H
#define CLUSTERING_RECT_H

#include "common.h"

/*
Function process:
+ Clustered the list of rects.
*/
extern int clusterRects(const WissenObjectRect *srcRect, const int count, const float eps, int *treeNode, int *labelsNode);

#endif // CLUSTERING_RECT_H

