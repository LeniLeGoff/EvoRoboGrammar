#include "robot_design/graph.h"

namespace robot_design{

void create_tree_from_graph(const Graph& g, NodeIndex root_node, Tree::Ptr &tree){
    tree->node = root_node;
    std::vector<NodeIndex> next_nodes;
    for(EdgeIndex ei = 0; ei < g.edges_.size(); ei++){
        // if(g.edges_[ei].head_ == root_node){
        //     if(!g.edges_[ei].attrs_.mirror_){
        //         tree->edges.push_back(ei);
        //         next_nodes.push_back(g.edges_[ei].tail_);
        //     }
        //     continue;
        // }
        if(g.edges_[ei].tail_ == root_node){
            if(!g.edges_[ei].attrs_.mirror_){
                tree->edges.push_back(ei);
                next_nodes.push_back(g.edges_[ei].head_);
            }
        }
    }
    for(size_t i = 0; i < next_nodes.size(); i++){
        Tree::Ptr child = std::make_shared<Tree>();
        robot_design::create_tree_from_graph(g,next_nodes[i], child);
        tree->childs.push_back(child);
    }
}

int tree_edit_distance(const Graph &g1, const Graph &g2){
    NodeIndex root_node1, root_node2;
    root_node1 = find_root_node(g1);
    root_node2 = find_root_node(g2);
    Tree::Ptr tree1 = std::make_shared<Tree>();
    Tree::Ptr tree2 = std::make_shared<Tree>();
    robot_design::create_tree_from_graph(g1,root_node1,tree1);
    robot_design::create_tree_from_graph(g2,root_node2,tree2);

    return ted_rec(g1,g2,tree1,tree2);
}

int ted_rec(const Graph &g1, const Graph &g2,const Tree::Ptr &t1,const Tree::Ptr &t2){
    int dist = 0;
    if(t1 == nullptr && t2 == nullptr)
        return dist;
    if(t1 == nullptr){//arrived at the end of tree 1
        dist += 1; //node creation
        dist += t2->edges.size(); //edges addition
        for(const Tree::Ptr& child: t2->childs)
            dist += robot_design::ted_rec(g1,g2,nullptr,child);
        return dist;
    }
    if(t2 == nullptr){//arrived at the end of tree 2
        dist += 1; //node creation
        dist += t1->edges.size(); //edges addition
        for(const Tree::Ptr& child: t1->childs)
            dist += robot_design::ted_rec(g1,g2,nullptr,child);
        return dist;
    }

    //compare nodes type (swap node type op)
    if(g1.nodes_.at(t1->node).attrs_.label_ != g2.nodes_.at(t2->node).attrs_.label_)
        dist++;
    //compare nodes connectivity (add or delete edge op)
    dist += abs(static_cast<int>(t1->edges.size() - t2->edges.size()));
    //compare edges type (swap edge type op)
    const std::vector<EdgeIndex> &shortest = t1->edges.size() <= t2->edges.size() ? t1->edges : t2->edges;
    const std::vector<EdgeIndex> &largest = t1->edges.size() <= t2->edges.size() ? t2->edges : t1->edges;
    const robot_design::Graph g_shortest = t1->edges.size() <= t2->edges.size() ? g1 : g2;
    const robot_design::Graph g_largest = t1->edges.size() <= t2->edges.size() ? g2 : g1;
    for(EdgeIndex ei: shortest){
        bool found_similar = false;
        for(EdgeIndex ej: largest){
            if(g_shortest.edges_[ei].attrs_.label_ == g_largest.edges_[ej].attrs_.label_){
                found_similar = true;
                break;
            }
        }
        dist += found_similar ? 0 : 1;
    }
    //child pairing with most similar for further comparison
    std::vector<EdgeIndex> edges1 = t1->edges;
    std::vector<EdgeIndex> edges2 = t2->edges;
    std::vector<std::pair<int,int>> child_pairing;
    auto itei = edges1.begin();
    int i = 0, j = 0;
    for(size_t ei = 0; ei < edges1.size(); ei++){
        int eidx = edges1[ei];
        if(edges2.empty()){
            child_pairing.push_back(std::make_pair(i++,-1));
            edges1.erase(edges1.begin() + ei--);
            break;
        }
        for(size_t ej = 0; ej < edges2.size(); ej++){
            int ejdx = edges2[ej];
            if(g1.edges_[eidx].attrs_.label_ == g2.edges_[ejdx].attrs_.label_){//if exactly the same
                child_pairing.push_back(std::make_pair(i++,j++));
                edges1.erase(edges1.begin() + ei--);
                edges2.erase(edges2.begin() + ej--);
                break;
            }
            std::stringstream sstr1(g1.edges_[eidx].attrs_.label_);
            std::string type1;
            std::getline(sstr1,type1,'_');
            std::stringstream sstr2(g2.edges_[ejdx].attrs_.label_);
            std::string type2;
            std::getline(sstr2,type2,'_');
            if(type1 == type2){ //if same type
                child_pairing.push_back(std::make_pair(i++,j++));
                edges1.erase(edges1.begin() + ei--);
                edges2.erase(edges2.begin() + ej--);
                break;
            }
        }
    }
    //second pass to pair non-similar remaining edges;
    for(size_t ei = 0; ei < edges1.size(); ei++){
        for(size_t ej = 0; ei < edges2.size(); ej++){
            if(!edges1.empty() && !edges2.empty()){
                child_pairing.push_back(std::make_pair(i++,j++));
                edges1.erase(edges1.begin() + ei--);
                edges2.erase(edges2.begin() + ej--);

            }else if(!edges1.empty()){
                child_pairing.push_back(std::make_pair(-1,j++));
                edges2.erase(edges2.begin() + ej--);
            }else if(!edges2.empty()){
                child_pairing.push_back(std::make_pair(i++,-1));
                edges1.erase(edges1.begin() + ei--);
            }
        }
    }

    //compute tree distance between paired childs
    for(const auto& pair: child_pairing){
        const Tree::Ptr &child1 = pair.first >= 0 ? t1->childs[pair.first] : nullptr;
        const Tree::Ptr &child2 = pair.second >= 0 ? t2->childs[pair.second] : nullptr;
        dist += robot_design::ted_rec(g1,g2,child1,child2);
    }
    return dist;
}
}
