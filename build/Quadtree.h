#include <iostream>
#include <cmath>
#include <vector>

// insert : O(log N)
// search : O(log N)

struct Corner{
    double u; 
    double v;
    // Constructor
    Corner(){
        u = 0;
        v = 0;
    }
    Corner(double u_, double v_) {
        u = u_;
        v = v_;
    } 
};

struct Point{
    double u;
    double v;
    int    idx;    // 이 점의 고유 정보. 몇번째 포인트인지 indexing 해주는것.
    void*  node_ptr; // 이 점이 속한 node의 pointer. Point q에 대한 matching p_match 에 대해서만 할당해준다.
    Point(){
        u = 0;
        v = 0;
        idx = 0;
        node_ptr = NULL;
    }
    Point(double u_, double v_, int idx_){
        u    = u_;
        v    = v_;
        idx  = idx_;
        node_ptr = NULL;
    }
};

struct Node{
    Corner topleft;
    Corner botright;
    Corner center;
    Point  pt;

    int    depth;
    bool   isleaf;

    std::vector<Point> pts; // the number of point in this ...
    std::vector<Node*> leaf_nodes;

    Node* parent; // if it is the root node, assign NULL.

    Node* child_tl; // north west
    Node* child_tr; // north east
    Node* child_bl; // south west
    Node* child_br; // south east

    Node(Corner topleft_, Corner botright_, Point pt_, Node* parent_, int depth_){
        topleft  = topleft_;
        botright = botright_;
        center   = Corner( (topleft.u + botright.u) * 0.5, (topleft.v + botright.v) * 0.5);
        pt       = pt_; // leaf node가 아닐 경우, 임시로 저장하기 위해 사용하는 것.
        depth    = depth_;

        parent   = parent_;
        child_tl = NULL;
        child_tr = NULL;
        child_bl = NULL;
        child_br = NULL;
        
        isleaf   = true; // 새로 생성되는 node는 당연히 leaf이다. 
        pts.reserve(50);
    }
};


class QuadTree{
public: // variables
    Node*  root; 
    Corner topleft_root;
    Corner botright_root;
    int max_depth; // This is the maximum depth of the leaf node.
    double eps;    // approximated quadtree 가 되기 위한 값. default = 0. 권장, 0.2.
    double scale;  // 1-eps.

public: // methods
    QuadTree(){};
    QuadTree(double n_rows_, double n_cols_, int max_depth_, double eps_); 
    QuadTree(std::vector<Point> pts_input, double n_rows_, double n_cols_, int max_depth_, double eps_);  
    void createRoot(double n_rows_, double n_cols_);
    bool inBoundary(Point p);
    bool BWBTest(Node* node, Point q, double dist);
    void insert(Node* node, Point p, int depth);
    Point cachedNNSearch(Node* node, Point q);
    void cachedNNSearchMultiple(std::vector<Point> qs, std::vector<void*> cached_nodes, std::vector<int>& idxs);
};


/**
 * ----------------------------------------------------------------------------
 * 
 * 
 *                                IMPLEMENTATION
 * 
 * 
 * ----------------------------------------------------------------------------
 */

inline double distanceEuclidean(Point p, Point q){
    return (p.u-q.u)*(p.u-q.u)+(p.v-q.v)*(p.v-q.v);
};
QuadTree::QuadTree(double n_rows_, double n_cols_, int max_depth_, double eps_) {
    QuadTree::createRoot(n_rows_, n_cols_);
    max_depth = max_depth_;
    eps       = eps_;
    scale     = 1.0 - eps;
};

QuadTree::QuadTree(std::vector<Point> pts_input, double n_rows_, double n_cols_, int max_depth_, double eps_) {
    QuadTree::createRoot(n_rows_, n_cols_);
    max_depth = max_depth_;
    eps       = eps_;
    scale     = 1.0 - eps;

    for(int i = 0; i < pts_input.size(); i++){
        QuadTree::insert(root,pts_input[i], 0);
    }
};


void QuadTree::createRoot(double n_rows_, double n_cols_){
    root           = new Node(Corner(0,0), Corner(n_cols_, n_rows_), Point(-1,-1,-1), NULL, 0); // this is root node.
    topleft_root   = Corner(0,0);
    botright_root  = Corner(n_cols_, n_rows_);
    root->isleaf   = false;
};

bool QuadTree::inBoundary(Point p){ // 입력해주는 점이 global boundary안에 있는지 확인한다.
    return (p.u >= topleft_root.u && p.u <= botright_root.u && p.v >= topleft_root.v && p.v <= botright_root.v);
};

bool QuadTree::BWBTest(Node* node, Point q, double dist){ // leaf에서 이거 통과하면 지금의 매칭이 리얼 매칭이다.
    // leaf가 아닌경우에는 BOB test에서 활용 될 것이다.
    dist = scale*dist;
    return (node->topleft.u <= q.u - dist && node->botright.u >= q.u + dist
         && node->topleft.v <= q.v - dist && node->botright.v >= q.v + dist);
};

void QuadTree::insert(Node* node, Point p, int depth){ // 맨 처음 넣어주는 Node*는 root 이다.
    if(depth == max_depth){ // 최대 깊이이면, 더이상 진행 불가하고, pts에 넣어준다.
        node->isleaf = true;
        p.node_ptr = (void*)node; // 이제부터 이 점의 소속은 node이다. 
        node->pts.push_back(p);
        return;
    }
    else{ // 최대 깊이가 아니다. 아래 node가 더 있을 수 있다. (해당 라인은 최대 depth - 1 까지만 도달가능.)
        if(node->isleaf) {// 현재 노드가 leaf이면 leaf의 점도 같이 가지고 내려간다. 
            node->isleaf = false; // 이제부터는 자식 노드가 생기므로 더이상 leaf가 아니다.
            // 1) node->pt 를 여행시켜준다. 자식이 없으므로 자식 생성해줘야하고, 바로 아래 child로 갈 뿐이다.
            if(node->center.u >= node->pt.u ) { // west(left), 당연히 아무것도 없으니까 새노드 생성.
                if(node->center.v >= node->pt.v){ // north west
                    node->child_tl = new Node(node->topleft, node->center, node->pt, node, depth + 1);
                    node->child_tl->pt.node_ptr = (void*)node->child_tl;
                }
                else{// south west
                    node->child_bl = new Node(Corner(node->topleft.u, node->center.v), Corner(node->center.u, node->botright.v), node->pt, node, depth + 1);
                    node->child_bl->pt.node_ptr = (void*)node->child_bl;
                }
            }
            else{ // east (right), 당연히 아무것도 없으니까 새노드 생성.
                if(node->center.v >= node->pt.v){ // north east
                    node->child_tr = new Node(Corner(node->center.u, node->topleft.v), Corner(node->botright.u, node->center.v), node->pt, node, depth + 1);
                    node->child_tr->pt.node_ptr = (void*)node->child_tr;
                }
                else{// south east
                    node->child_br = new Node(node->center, node->botright, node->pt, node, depth + 1);
                    node->child_br->pt.node_ptr = (void*)node->child_br;
                }
            }
      
            // 2) p를 여행시켜준다.
            if(node->center.u >= p.u ) { // west(left)
                if(node->center.v >= p.v){ // north west
                    if(node->child_tl == NULL){// 아래에 아무것도 안들어있다면, 넣어주고 leaf가 된다.
                        node->child_tl = new Node(node->topleft, node->center, p, node, depth + 1);
                        node->child_tl->pt.node_ptr = (void*)node->child_tl;
                    }
                    else // 자식노드가 있다면, insert 함수 재호출.
                        QuadTree::insert(node->child_tl, p, depth + 1);
                }
                else{ // south west
                    if(node->child_bl == NULL){ // 아래에 아무것도 안들어있다면, 넣어주고 leaf가 된다.
                        node->child_bl = new Node(Corner(node->topleft.u, node->center.v), Corner(node->center.u, node->botright.v), p, node, depth + 1);
                        node->child_bl->pt.node_ptr = (void*)node->child_bl;
                    }
                    else // 자식노드가 있다면, insert 함수 재호출.
                        QuadTree::insert(node->child_bl, p, depth + 1);
                }
            }
            else { // east (right)
                if(node->center.v >= p.v){ // north east
                    if(node->child_tr == NULL){// 아래에 아무것도 안들어있다면, 넣어주고 leaf가 된다.
                        node->child_tr = new Node(Corner(node->center.u, node->topleft.v), Corner(node->botright.u, node->center.v), p, node, depth + 1);
                        node->child_tr->pt.node_ptr = (void*)node->child_tr;
                    }
                    else // 자식노드가 있다면, insert 함수 재호출.
                        QuadTree::insert(node->child_tr, p, depth + 1);
                }
                else{ // south east
                    if(node->child_br == NULL){// 아래에 아무것도 안들어있다면, 넣어주고 leaf가 된다.
                        node->child_br = new Node(node->center, node->botright, p, node, depth + 1);
                        node->child_br->pt.node_ptr = (void*)node->child_br;
                    }
                    else // 자식노드가 있다면, insert 함수 재호출.
                        QuadTree::insert(node->child_br, p, depth + 1);
                }
            }
        }
        else{ // 현재 노드가 leaf가 아니면 p만 가지고 아래로 여행하자. 
            if(node->center.u >= p.u ) { // west(left)
                if(node->center.v >= p.v){ // north west
                    if(node->child_tl == NULL){ // 아래에 아무것도 안들어있다면, 넣어주고 leaf가 된다.
                        node->child_tl = new Node(node->topleft, node->center, p, node, depth + 1);
                        node->child_tl->pt.node_ptr = (void*)node->child_tl;
                        return;
                    }
                    else // 자식노드가 있다면, insert 함수 재호출.
                        QuadTree::insert(node->child_tl, p, depth + 1);
                }
                else{ // south west
                    if(node->child_bl == NULL) {// 아래에 아무것도 안들어있다면, 넣어주고 leaf가 된다.
                        node->child_bl = new Node(Corner(node->topleft.u, node->center.v), Corner(node->center.u, node->botright.v), p, node, depth + 1);
                        node->child_bl->pt.node_ptr = (void*)node->child_bl;
                        return;
                    }
                    else // 자식노드가 있다면, insert 함수 재호출.
                        QuadTree::insert(node->child_bl, p, depth + 1);
                }
            }
            else { // east (right)
                if(node->center.v >= p.v){ // north east
                    if(node->child_tr == NULL){ // 아래에 아무것도 안들어있다면, 넣어주고 leaf가 된다.
                        node->child_tr = new Node(Corner(node->center.u, node->topleft.v), Corner(node->botright.u, node->center.v), p, node, depth + 1);
                        node->child_tr->pt.node_ptr = (void*)node->child_tr;
                        return;
                    }
                    else // 자식노드가 있다면, insert 함수 재호출.
                        QuadTree::insert(node->child_tr, p, depth + 1);
                }
                else{ // south east
                    if(node->child_br == NULL){ // 아래에 아무것도 안들어있다면, 넣어주고 leaf가 된다.
                        node->child_br = new Node(node->center, node->botright, p, node, depth + 1);
                        node->child_br->pt.node_ptr = (void*)node->child_br;
                        return;
                    }
                    else // 자식노드가 있다면, insert 함수 재호출.
                        QuadTree::insert(node->child_br, p, depth + 1);
                }
            }
        }
    }
};

void QuadTree::cachedNNSearchMultiple(std::vector<Point> qs, std::vector<void*> cached_nodes, std::vector<int>& idxs){
    int npoints = qs.size();
    Point p_match;
    for(int i = 0; i < npoints; i++){
        p_match = QuadTree::cachedNNSearch(root, qs[i]);
        idxs.push_back(p_match.idx);
        //cached_nodes[i] = (void*)p_match.node_ptr;
    }
};

Point QuadTree::cachedNNSearch(Node* node, Point q){
    Node* node_now = node;
    Point p_match;
    double min_dist = 1e11;

    // (1) 입력해준 node에서 가장 가까운 점을 찾는다.
    double dist_temp = 0;
    if(node->depth == 0)
        dist_temp = 0;
    else{
        if(node->depth == max_depth){
            for(int i =0; i < node->pts.size(); i++){
                dist_temp = distanceEuclidean(q, node->pts[i]);
                if(min_dist > dist_temp){
                    min_dist = dist_temp;
                    p_match  = node->pts[i];
                    p_match.node_ptr = (void*)node;
                }
            }
        }
        else{ // max_depth leaf가 아닌경우.
            dist_temp = distanceEuclidean(q, node->pt);
            if(min_dist > dist_temp){
                min_dist = dist_temp;
                p_match  = node->pt;
                p_match.node_ptr = (void*)node;
            }

        }
    }
    
    // (2) BWB == true가 될 때 까지 parent로 올라간다.
    if(node_now->depth !=0){ // root 인 경우는 패스.
        while( !QuadTree::BWBTest(node_now, q, min_dist) ) // BWBTest가 될 때까지 위로 올라간다.
            node_now = node_now->parent;
    }

    // (3) q를 contain 하는 node로 다시 찾아 내려간다. 가는 동안 BOB 테스트 같은건 필요없다.
    while( !node_now->isleaf ){ // leaf node가 될때까지 반복한다.
        if(node_now->center.u >= q.u){ // left
            if(node_now->center.v >= q.v) // left top
                node_now = node_now->child_tl;
            else // left bot
                node_now = node_now->child_bl;
        }
        else{ // right
            if(node_now->center.v >= q.v) // right top
                node_now = node_now->child_tr;
            else // right bot
                node_now = node_now->child_br;
        }
    }

    // (4) 최종 도달한 node에서 다시 최근접점을 찾는다. 
    if(node_now->depth == max_depth){
        for(int i =0; i < node_now->pts.size(); i++){
            dist_temp = distanceEuclidean(q, node_now->pts[i]);
            if(min_dist > dist_temp){
                min_dist = dist_temp;
                p_match  = node_now->pts[i];
                p_match.node_ptr = (void*)node_now;
            }
        }
    }
    else{ // max_depth leaf가 아닌경우.
        dist_temp = distanceEuclidean(q, node_now->pt);
        if(min_dist > dist_temp){
            min_dist = dist_temp;
            p_match  = node_now->pt;
            p_match.node_ptr = (void*)node_now;
        }
    }

    // 끝.
    return p_match;
};





/* // 사실 필요한건지 모르겠다. 
bool QuadTree::BOBTest(Node* node, Point q, double ball_radius){ // BOB? BWB가 만족되지 않으면 계속 올라가는데, 
    // determine the location of q relative to interesting node boundary.
    if(q.u <= node->topleft.u){ // left
        if(q.v <= node->topleft.v){ // left top( north west 방향에 점이 위치)
            dist_temp = (node->topleft.u - q.u)*(node->topleft.u - q.u) + (node->topleft.v - q.v)*(node->topleft.v - q.v);
            if(ball_radius >= dist_temp) // ball_radius 가 더 크면 overlap.
                return true;
            else
                return false;
        }
        else{ // left bot( south west 방향에 점이 위치)
            dist_temp = (node->topleft.u - q.u)*(node->topleft.u - q.u) + (node->botright.v - q.v)*(node->botright.v - q.v);
            if(ball_radius >= dist_temp) // ball_radius 가 더 크면 overlap.
                return true;
            else
                return false;
        }
    }
    else{ // right
        if(q.v <= node->topleft.v){ // right top( north east 방향에 점이 위치)
            dist_temp = (node->botright.u - q.u)*(node->botright.u - q.u) + (node->topleft.v - q.v)*(node->topleft.v - q.v);
            if(ball_radius >= dist_temp) // ball_radius 가 더 크면 overlap.
                return true;
            else
                return false;
        }
        else{ // right bot( south east 방향에 점이 위치)
            dist_temp = (node->botright.u - q.u)*(node->botright.u - q.u) + (node->botright.v - q.v)*(node->botright.v - q.v);
            if(ball_radius >= dist_temp) // ball_radius 가 더 크면 overlap.
                return true;
            else
                return false;
        }
    }
}; */

/* bool QuadTree::nearestNeighborSearch(Node* node, Point q, Point& p_match, double& min_dist){ // 하나의 query point q에 대해서 검색한다.
    // min_dist 의 초기값은 infinite이다.
    if(node->isleaf){ // leaf node이면 비교해 볼 point들이 존재한다.
        if(node->depth == max_depth){ // 최하층이면 pts. (Linear search).
            double dist_temp = 0;
            for(int i = 0; i < node->pts.size(); i++){
                dist_temp = distanceEuclidean(q, node->pts[i]);
                if(*min_dist > dist_temp){
                    *min_dist = dist_temp;
                    *p_match  = node->pts[i];
                    p_match->node_ptr = node;
                }
            }
        }
        else{ // 최하층이 아닌 leaf인 경우, 
            double dist_temp = 0;
            dist_temp = distanceEuclidean(q,node->pt);
            if(*min_dist > dist_temp){
                *min_dist = dist_temp;
                *p_match  = node->pt;
                p_match->node_ptr = node;
            }
        }

        if(BWBTest(node, q, *min_dist)){
            return true; // 완전히 모든 연산이 끝났다. 계속 타고 나가면서 재귀를 collapse.
        }
        else{
            return false; // 아직 BWB 만족못했으니 다른 놈을 찾아본다. 
        }
    }
    else{ // leaf가 아닌경우, 아래로 traverse. 단 BOB test가 만족되는 경우에만 간다. 아닌 경우에는 갈 이유가 없다.
        if(node->center.u >= q.u){ // left
            if(node->center.v >= q.v){ // left top
                if(BOBTest(node->child_tl, q, *min_dist))
                    if(nearestNeighborSearch(node->child_tl, q, min_dist))
                        return true;
            }
            else{ // left bot
                if(BOBTest(node->child_bl, q, *min_dist))
                    if(nearestNeighborSearch(node->child_bl, q, min_dist))
                        return true;
            }
        }
        else{
            if(node->center.v >= q.v){ // right top
                if(BOBTest(node->child_tr, q, *min_dist))
                    if(nearestNeighborSearch(node->child_tr, q, min_dist))
                        return true;
            }
            else { // right bot
                if(BOBTest(node->child_br, q, *min_dist))
                    if(nearestNeighborSearch(node->child_br, q, min_dist))
                        return true;
            }
        }
    }
};
 */