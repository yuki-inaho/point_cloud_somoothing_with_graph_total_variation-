#include "header.h"

typedef std::pair<int, int> HULL_TUPLE;

//std::vector<HULL_TUPLE>

template<typename T>
static inline double Lerp(T v0, T v1, T t)
{
    return (1 - t)*v0 + t*v1;
}

template<typename T>
static inline T Quantile(const std::vector<T>& inData, const T& prob)
{
    if (inData.empty())
    {
        T temp;
        return temp;
    }

    if (1 == inData.size())
    {
        return inData[0];
    }

    std::vector<T> data = inData;
    std::sort(data.begin(), data.end());

    T poi = Lerp<T>(-0.5, data.size() - 0.5, prob);
    size_t left = std::max(int64_t(std::floor(poi)), int64_t(0));
    size_t right = std::min(int64_t(std::ceil(poi)), int64_t(data.size() - 1));

    T datLeft = data.at(left);
    T datRight = data.at(right);
    T quantile = Lerp<T>(datLeft, datRight, poi - left);
    return quantile;
}

double 
calcDistFromCenter(pcl::PointXYZ point_center, int idx, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    double diff_x = cloud->points[idx].x - point_center.x;
    double diff_y = cloud->points[idx].y - point_center.y;
    double diff_z = cloud->points[idx].z - point_center.z;
    double diff = std::sqrt(diff_x*diff_x + diff_y*diff_y + diff_z*diff_z);
    return diff;
}

double 
calcArea(pcl::PointXYZ point_center, pcl::PointXYZ p, pcl::PointXYZ q){
    double diff_center2p_x = p.x - point_center.x;
    double diff_center2p_y = p.y - point_center.y;
    double diff_center2p_z = p.z - point_center.z;

    double diff_center2q_x = q.x - point_center.x;
    double diff_center2q_y = q.y - point_center.y;
    double diff_center2q_z = q.z - point_center.z;

    double outer_x = diff_center2p_y *diff_center2q_z - diff_center2p_z *diff_center2q_y;
    double outer_y = - diff_center2p_x *diff_center2q_z + diff_center2p_z *diff_center2q_x;
    double outer_z = diff_center2p_x *diff_center2q_y - diff_center2p_y *diff_center2q_x;

    return 0.5 *std::sqrt(outer_x*outer_x + outer_y*outer_y + outer_z*outer_z);
}

/*Bipartite*/
std::vector<HULL_TUPLE>
calcHull4SurfaceNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bipartite){
    std::vector<HULL_TUPLE> hull_indices;

    int K = 30;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud_bipartite);

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    for (int idx = 0; idx < static_cast<int> (cloud->points.size ()); ++idx){
        HULL_TUPLE _tmp;
        hull_indices.push_back(_tmp);
    }

    int threads_ = 8;
    //#pragma omp parallel for shared (output) private (pointIdxNKNSearch, pointNKNSquaredDistance) num_threads(threads_)
    #pragma omp parallel for private (pointIdxNKNSearch, pointNKNSquaredDistance) 
    for (int idx = 0; idx < static_cast<int> (cloud->points.size ()); ++idx)
    {
        pcl::PointXYZ point_center = cloud->points[idx];
        //if(search_method_surface_ (*cloud, idx, K, pointIdxNKNSearch, pointNKNSquaredDistance)>0)
        if ( kdtree.nearestKSearch (point_center, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nn (new pcl::PointCloud<pcl::PointXYZ>);

            std::unordered_map<int, int> map_local2global;
            for (int k = 0; k < pointIdxNKNSearch.size (); ++k){
                pcl::PointXYZ point;
                point.x = cloud->points[pointIdxNKNSearch[k]].x;
                point.y = cloud->points[pointIdxNKNSearch[k]].y;
                point.z = cloud->points[pointIdxNKNSearch[k]].z;
                cloud_nn->points.push_back(point);
                map_local2global.insert(std::pair<int, int>(k, pointIdxNKNSearch[k]));
            }
            
            std::vector<double> dist_vec;
            for (int j = 0; j < cloud_nn->points.size (); ++j){
                dist_vec.push_back(calcDistFromCenter(point_center, j, cloud_nn));
            }

            double dist_quantile = Quantile<double>(dist_vec, 0.50);
            std::vector<int> indices_dist_inlier;

            int arg_quantile;
            double _min_diff_quant = dist_quantile;

            for (int j = 0; j < cloud_nn->points.size (); ++j){
                if(dist_vec[j] <= dist_quantile) indices_dist_inlier.push_back(j);
                double _dist = dist_quantile - dist_vec[j];
                if(_dist <= _min_diff_quant){
                    arg_quantile = j;
                    _min_diff_quant = _dist;
                } 
            }

            //double area = calcArea(point_center, cloud_nn->points[arg_quantile], cloud_nn->points[0]);

            double max_area = 0;
            int arg_max_area = 0;
            for (int l = 0; l < indices_dist_inlier.size (); ++l){
                if(l == arg_quantile) continue;
                double area = calcArea(point_center, cloud_nn->points[arg_quantile], cloud_nn->points[l]);
                if(max_area < area){
                    max_area = area;
                    arg_max_area = l;
                }
            }
            //cout << idx << " " << map_local2global.at(arg_quantile) << " " << map_local2global.at(arg_max_area) << endl;
            HULL_TUPLE tuple = std::pair<int, int>(map_local2global.at(arg_quantile), map_local2global.at(arg_max_area));
            hull_indices[idx] = tuple;
        }
    }

    return hull_indices;
}

void
Hull2NormalParam(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bipartite,
//            const std::vector<HULL_TUPLE> &hull_indices, pcl::PointCloud<pcl::Normal>::Ptr normals){
                const std::vector<HULL_TUPLE> &hull_indices, 
                //Eigen::SparseMatrix<float, Eigen::ColMajor> &_A ,
                std::vector<Eigen::MatrixXf> &_A ,
                Eigen::VectorXf &_p, Eigen::VectorXf &_b, Eigen::VectorXf &_n){

    int n_point = cloud->points.size();
    //Eigen::SparseMatrix<float, Eigen::ColMajor> A(n_point*3, n_point*3);
    Eigen::MatrixXf A(3, 3);
    Eigen::VectorXf p(n_point*3);    
    Eigen::VectorXf b(n_point*3);
    Eigen::VectorXf n(n_point*3);

    std::vector<Eigen::Triplet<float>> tripletVec;
    for(int i=0;i<cloud->points.size();i++){
        pcl::PointXYZ point_center, point_p, point_q;
        point_center = cloud->points[i];
        point_p = cloud->points[hull_indices[i].first];
        point_q = cloud->points[hull_indices[i].second];

        p(i*3 + 0) = point_center.x;
        p(i*3 + 1) = point_center.y;
        p(i*3 + 2) = point_center.z;

        float vec_p2center_x = point_center.x - point_p.x;
        float vec_p2center_y = point_center.y - point_p.y;
        float vec_p2center_z = point_center.z - point_p.z;

        float vec_p2q_x = point_q.x - point_p.x;
        float vec_p2q_y = point_q.y - point_p.y;
        float vec_p2q_z = point_q.z - point_p.z;

        float outer_x = vec_p2center_y * vec_p2q_z - vec_p2center_z * vec_p2q_y;
        float outer_y = - vec_p2center_x * vec_p2q_z + vec_p2center_z * vec_p2q_x;
        float outer_z = vec_p2center_x * vec_p2q_y - vec_p2center_y * vec_p2q_x;

        float norm = std::sqrt(outer_x*outer_x + outer_y*outer_y + outer_z*outer_z);
        float n_x = outer_x/norm;
        float n_y = outer_y/norm;
        float n_z = outer_z/norm;

        float norm_center = std::sqrt(point_center.x*point_center.x + point_center.y*point_center.y + point_center.z*point_center.z);
        float to_center_x = point_center.x/norm_center;
        float to_center_y = point_center.y/norm_center;
        float to_center_z = point_center.z/norm_center;
        float inner_product = n_x * to_center_x + n_y * to_center_y + n_z * to_center_z;

        float alpha = 1;
        if(inner_product < 0){
            alpha = -1;
        }
/*
        tripletVec.push_back( Eigen::Triplet<float>(i*3 + 0,i*3 + 0,0) );
        tripletVec.push_back( Eigen::Triplet<float>(i*3 + 1,i*3 + 1,0) );
        tripletVec.push_back( Eigen::Triplet<float>(i*3 + 2,i*3 + 2,0) );
        tripletVec.push_back( Eigen::Triplet<float>(i*3 + 0,i*3 + 1,vec_p2q_z/norm*alpha) );
        tripletVec.push_back( Eigen::Triplet<float>(i*3 + 1,i*3 + 0,-vec_p2q_z/norm*alpha) );
        tripletVec.push_back( Eigen::Triplet<float>(i*3 + 0,i*3 + 2,-vec_p2q_y/norm*alpha) );
        tripletVec.push_back( Eigen::Triplet<float>(i*3 + 2,i*3 + 0,vec_p2q_y/norm*alpha) );
        tripletVec.push_back( Eigen::Triplet<float>(i*3 + 1,i*3 + 2,vec_p2q_x/norm*alpha) );
        tripletVec.push_back( Eigen::Triplet<float>(i*3 + 2,i*3 + 1,-vec_p2q_x/norm*alpha) );
        */

        A(0,0) = 0; A(1,1) = 0; A(2,2) = 0; 
        A(0,1) = vec_p2q_z/norm*alpha; A(1,0) = -A(0,1); 
        A(0,2) = -vec_p2q_y/norm*alpha; A(2,0) = -A(0,2); 
        A(1,2) = vec_p2q_x/norm*alpha; A(2,1) = -A(1,2); 

        b(i*3 + 0)  =  (- point_p.y * point_q.z  + point_p.z * point_q.y)/norm*alpha;
        b(i*3 + 1)  =  (point_p.x * point_q.z  - point_p.z * point_q.x)/norm*alpha;
        b(i*3 + 2)  =  (- point_p.x * point_q.y  + point_p.y * point_q.x)/norm*alpha;

        Eigen::VectorXf __p(3);    
        __p(0) = point_center.x;
        __p(1) = point_center.y;
        __p(2) = point_center.z;

        Eigen::VectorXf _b(3);    
        _b(0) = b(i*3 + 0);
        _b(1) = b(i*3 + 1);
        _b(2) = b(i*3 + 2);

        _A.push_back(A);
        Eigen::VectorXf __n = A*__p + _b;
        n(i*3 + 0) = __n(0);
        n(i*3 + 1) = __n(1);
        n(i*3 + 2) = __n(2);
    }
    //A.setFromTriplets(tripletVec.begin(), tripletVec.end());
    /*
    n = A*p + b;    
    for(int i=0;i<cloud->points.size();i++){
        pcl::Normal _normal;
        _normal.normal_x = n(i*3 + 0);
        _normal.normal_y = n(i*3 + 1);
        _normal.normal_z = n(i*3 + 2);
        normals->points.push_back(_normal);        
    } 
    */   
    //_A = A;
    _p = p;
    _b = b;
    _n = n;
}

void
getOptimizationParams(const std::vector<EdgeDesc> &mst, const Graph &g,
                    const Eigen::VectorXf &_p, const std::vector<Eigen::MatrixXf> &_A, const Eigen::VectorXf &_b, 
                    Eigen::SparseMatrix<float, Eigen::ColMajor> &_B, Eigen::VectorXf &_v, Eigen::VectorXf &_m, Eigen::VectorXf &w, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    int n_point = _A.size();

    std::forward_list<Eigen::SparseMatrix<float, Eigen::ColMajor>> subB_list;
    std::forward_list<Eigen::VectorXf> subb_list;

    //#pragma omp parallel for
    for (int m=0; m<mst.size();m++) {
        EdgeDesc e = mst[m];
        Eigen::SparseMatrix<float, Eigen::ColMajor> __B(n_point*3, n_point*3);
        Eigen::VectorXf __v(n_point*3);

        int source_ind = boost::source(e, g);
        int target_ind = boost::target(e, g);

        std::vector<Eigen::Triplet<float>> tripletVec;
        for(int i=0;i<3;i++){
            for(int j=0;j<3;j++){
                tripletVec.push_back( Eigen::Triplet<float>(source_ind*3 + i,source_ind*3 + j, _A[source_ind](i,j)));        
            }
        }

        for(int i=0;i<3;i++){
            for(int j=0;j<3;j++){
                tripletVec.push_back( Eigen::Triplet<float>(source_ind*3 + i,target_ind*3 + j, -_A[target_ind](i,j)));        
            }
        }
        __v(source_ind*3 + 0) = _b(source_ind*3 + 0) - _b(target_ind*3 + 0);
        __v(source_ind*3 + 1) = _b(source_ind*3 + 1) - _b(target_ind*3 + 1);
        __v(source_ind*3 + 2) = _b(source_ind*3 + 2) - _b(target_ind*3 + 2);

        float diff_x = _p(source_ind*3+0) - _p(target_ind*3+0);
        float diff_y = _p(source_ind*3+1) - _p(target_ind*3+1);
        float diff_z = _p(source_ind*3+2) - _p(target_ind*3+2);

        w(source_ind) = w(source_ind) + exp(- (diff_x*diff_x + diff_y*diff_y + diff_z*diff_z)/(2.0* 0.01*0.01));

        __B.setFromTriplets(tripletVec.begin(), tripletVec.end());
        subB_list.push_front(__B);
        subb_list.push_front(__v);
    }

    for (auto it=subB_list.begin(); it!=subB_list.end(); ++it) {
        _B = _B + *it;
    }

    for (auto it=subb_list.begin(); it!=subb_list.end(); ++it) {
        _v = _v + *it;
    }

    _m = _B*_p + _v;
}

void
OptimP(const Eigen::SparseMatrix<float, Eigen::ColMajor> &_B, const Eigen::VectorXf &_v, const Eigen::VectorXf &_m_init, const Eigen::VectorXf &_q, int n_point, Eigen::VectorXf &_p_new, const Eigen::VectorXf &w)
{
    float t=0.01;
    float gamma=0.05;
    float sigma = 5;

    Eigen::SparseMatrix<float, Eigen::ColMajor> left(n_point*3, n_point*3);
    for(int i=0;i<n_point*3;i++){
        left.coeffRef(i,i) = 2;
    }
    left = left + sigma*_B.transpose()*_B;

    Eigen::VectorXf _u(n_point*3);
    Eigen::VectorXf _p = _q;
    Eigen::VectorXf _m = _m_init;

    int n_iter = 30;
    for(int k=0;k<n_iter;k++){
        Eigen::VectorXf right;
        right = 2.0 * _q + sigma * _B.transpose()* (_m - _v - _u);

        Eigen::ConjugateGradient<Eigen::SparseMatrix<float>, Eigen::Lower|Eigen::Upper> cg;
        cg.compute(left);
        cg.setMaxIterations(1000);
        cg.setTolerance(1e-5);
        _p_new = cg.solveWithGuess(right, _p);

        Eigen::VectorXf _m_grad =  -sigma* (_B*_p_new + _v - _m + _u );

        Eigen::VectorXf _m_new(n_point*3);
        for(int j=0;j<n_point;j++){
            float _m_grad_x = _m_grad(j*3 + 0);
            float _m_new_x =  _m(j*3 + 0) + -t * _m_grad_x;
            if(_m_new_x > t*gamma*w(j)){
                _m_new_x = _m_new_x - t*gamma*w(j) ;
            }else if(_m_new_x < t*gamma*w(j)){
                _m_new_x = _m_new_x + t*gamma*w(j) ;
            }else{
                _m_new_x = 0;
            }
            float _m_grad_y = _m_grad(j*3 + 1);
            float _m_new_y =  _m(j*3 + 1) + -t * _m_grad_y;
            if(_m_new_y > t*gamma*w(j)){
                _m_new_y = _m_new_y - t*gamma*w(j) ;
            }else if(_m_new_y < t*gamma*w(j)){
                _m_new_y = _m_new_y + t*gamma*w(j) ;
            }else{
                _m_new_y = 0;
            }
            float _m_grad_z = _m_grad(j*3 + 2);
            float _m_new_z =  _m(j*3 + 2) + -t * _m_grad_z;
            if(_m_new_z > t*gamma*w(j)){
                _m_new_z = _m_new_z - t*gamma*w(j) ;
            }else if(_m_new_z < t*gamma*w(j)){
                _m_new_z = _m_new_z + t*gamma*w(j) ;
            }else{
                _m_new_z = 0;
            }

            _m_new(j*3 + 0) = _m_new_x;
            _m_new(j*3 + 1) = _m_new_y;
            _m_new(j*3 + 2) = _m_new_z;
        }

        Eigen::VectorXf _u_new = _u + (_B*_p_new + _v - _m_new );

        _p = _p_new;
        _m = _m_new;
        _u = _u_new;


        Eigen::VectorXf diff_qp = (_q - _p);
        float dsum = 0;
        for(int d = 0;d < n_point;d++){
            dsum += std::sqrt(diff_qp(d*3 + 0)*diff_qp(d*3 + 0)+diff_qp(d*3 + 1)*diff_qp(d*3 + 1)+diff_qp(d*3 + 2)*diff_qp(d*3 + 2));
        }
        //cout << dsum << endl;
/*
        for(int d = 0;d < n_point;d++){
            cout << _u(d*3 + 0) << " ";
            cout << _u(d*3 + 1) << " ";
            cout << _u(d*3 + 2) << endl;
        }*/

    }

}