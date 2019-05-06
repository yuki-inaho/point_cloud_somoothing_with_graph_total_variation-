#include "header.h"


using namespace std;
using namespace pcl;


void
deriveMinimumSpanningTree(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<EdgeDesc> &_mst, Graph &_g){
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud);

    int K = 5;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    int n_points = cloud->points.size();
    std::vector<Edge> edge_list;
    std::vector<float> edge_weight_list;    

    for(int i=0;i<n_points;i++){
        if ( kdtree.nearestKSearch (cloud->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            Edge _edge_ik, _edge_ki; 
            for (int k = 1; k < pointIdxNKNSearch.size (); k++){
                if(pointNKNSquaredDistance[k] == 0) continue;
                _edge_ik = Edge(i,pointIdxNKNSearch[k]);
                edge_list.push_back(_edge_ik);
                edge_weight_list.push_back(pointNKNSquaredDistance[k]); 
                _edge_ki = Edge(pointIdxNKNSearch[k], i);
                edge_list.push_back(_edge_ki);
                edge_weight_list.push_back(pointNKNSquaredDistance[k]);                
            }
        }
    }
    Graph g = Graph(edge_list.begin(), edge_list.end(), edge_weight_list.begin(), n_points);
    std::vector<EdgeDesc> mst;
    boost::kruskal_minimum_spanning_tree(g, std::back_inserter(mst));
    _mst = mst;
    _g = g;
}

void
BipartiteSeparation(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices &red_indices, pcl::PointIndices &blue_indices){
    std::vector<pcl::PointIndices> initial_cluster_indices;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud);

    int K = 30;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    int n_points = cloud->points.size();
    std::vector<Edge> edge_list;
    std::vector<float> edge_weight_list;    

    for(int i=0;i<n_points;i++){
        if ( kdtree.nearestKSearch (cloud->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            Edge _edge; 
            for (int k = 0; k < pointIdxNKNSearch.size (); ++k){
                _edge = Edge(i,pointIdxNKNSearch[k]);
                edge_list.push_back(_edge);
                edge_weight_list.push_back(pointNKNSquaredDistance[k]);                
            }
        }
    }
    Graph g = Graph(edge_list.begin(), edge_list.end(), edge_weight_list.begin(), initial_cluster_indices.size());
    std::vector<VertexDesc> p(boost::num_vertices(g));
    std::vector<float> d(boost::num_vertices(g));
    VertexDesc s = boost::vertex(0, g);

    boost::property_map<Graph, boost::edge_weight_t>::type weightmap = boost::get(boost::edge_weight, g);
    boost::property_map<Graph, boost::vertex_index_t>::type indexmap = boost::get(boost::vertex_index, g);
    boost::dijkstra_shortest_paths(g, s,
                            boost::predecessor_map(boost::make_iterator_property_map(p.begin(), boost::get(boost::vertex_index, g))).
                            distance_map(boost::make_iterator_property_map(d.begin(), boost::get(boost::vertex_index, g))));

    std::vector<int> indices(d.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(), [&d](float i1, float i2) {
        return d[i1] < d[i2];
    });

    for(int i=0;i<indices.size();i++){
        if(i % 2 == 0){
            red_indices.indices.push_back(p[i]);
        }else{
            blue_indices.indices.push_back(p[i]);
        }
    }

    /*
    std::cout << "distances and parents:" << std::endl;
    boost::graph_traits < Graph >::vertex_iterator vi, vend;
    for (boost::tie(vi, vend) = boost::vertices(g); vi != vend; ++vi) {
        //std::cout << "distance(" << *vi << ") = " << d[*vi] << ", ";
        //std::cout << "parent(" << *vi << ") = " << p[*vi] << std::endl;
    }
    */
}

