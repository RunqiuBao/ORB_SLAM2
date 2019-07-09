#ifndef MYSTRUCT_H
#define MYSTRUCT_H

struct bbox{
    int lux;
    int luy;
    int rlx;
    int rly;
};

struct MaskSet{//runqiu: the masks set for one frame
    vector<string> labels;
    vector<bbox> bboxes;
};

#endif
