#pragma once
#include"image.h"
#include"PointsContainer.h"
#include<memory>
#include<vector>

class CameraForMapping;

class Node{
  public:
    // ********** members **********
    Node* parent_;
    std::vector<Node*> children_;
};

class NodeActPtAct : public Node{
  public:
    // ********** members **********
    int max_ = 0;
    int minmax_ = 0;
    float h_ = 0;
};

class NodeActPtCoarseGen : public Node{
  public:
    // ********** members **********
    ActivePoint* coarse_act_pt_;

};



class Tree{
  // class CameraForMapping : public Camera{
  public:
    // ********** members **********
    int levels_;
    int factor_;
    std::vector<Node*> roots_;

    CameraForMapping* cam_;

    // ********** constructor **********
    // tree generation from image
    Tree(int levels, int factor, CameraForMapping* cam, int mode):
    levels_(levels),
    factor_(factor),
    cam_(cam)
    { }

    // ********** methods **********
    void checkTreeValidity();
    // addRoot()
};

class TreeCandSel : public Tree{
  // ********** members **********

  // ********** constructor **********

  // ********** methods **********
  void init();

};

class TreeActPtAct : public Tree{
  // ********** members **********

  // ********** constructor **********

  // ********** methods **********
  void init();

};

class TreeActPtCoarseGen : public Tree{
  // ********** members **********

  // ********** constructor **********

  // ********** methods **********
  void init();
};
