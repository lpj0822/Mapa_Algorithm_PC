#ifndef SORT_ALGORITHM_H
#define SORT_ALGORITHM_H

#include "common.h"

/*
I/O:	    Name		    Type	     		  Content

[in/out]	data		    int*		          input data.
[in]	    n		        int*		          data num.

Realized function:
+ sort the int in decreasing order;
*/
void binSort_INT(int *data, int n);

/*
I/O:	    Name		    Type	     		  Content

[in/out]	data		    float*		      input data.
[in]	    n		        int*		          data num.

Realized function:
+ sort the float in decreasing order;
*/
void binSort_FLOAT(float *data, int n);

/*
I/O:	    Name		    Type	     		  Content

[in/out]	data		    AdasCorner*		      input corner.
[in]	    n		        int*		          corner num.

Realized function:
+ sort the corner in decreasing order by data.val;
*/
void binSort_Corner(AdasCorner *data, int n);

#endif //SORT_ALGORITHM_H