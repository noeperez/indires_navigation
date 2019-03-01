#ifndef RRT_NEAREST_NEIGHBORS_FLANN_
#define RRT_NEAREST_NEIGHBORS_FLANN_

#include <rrt_planners/NearestNeighbors.h>
#include <boost/shared_ptr.hpp>
#include <exception>

#include <flann/flann.hpp>


    /** \brief Wrapper class to allow FLANN access to the
        NearestNeighbors::distFun_ callback function
    */
    template<typename _T>
    class FLANNDistance
    {
    public:
        typedef _T ElementType;
        typedef double ResultType;

        FLANNDistance(const typename NearestNeighbors<_T>::DistanceFunction& distFun)
            : distFun_(distFun)
        {
        }

        template <typename Iterator1, typename Iterator2>
        ResultType operator()(Iterator1 a, Iterator2 b,
            size_t /*size*/, ResultType /*worst_dist*/ = -1) const
        {
            return distFun_(*a, *b);
        }
    protected:
        const typename NearestNeighbors<_T>::DistanceFunction& distFun_;
    };

    /** \brief Wrapper class for nearest neighbor data structures in the
        FLANN library.

        See:
        M. Muja and D.G. Lowe, "Fast Approximate Nearest Neighbors with
        Automatic Algorithm Configuration", in International Conference
        on Computer Vision Theory and Applications (VISAPP'09), 2009.
        http://people.cs.ubc.ca/~mariusm/index.php/FLANN/FLANN
    */
    template<typename _T, typename _Dist = FLANNDistance<_T> >
    class NearestNeighborsFLANN : public NearestNeighbors<_T>
    {
    public:
		//--Added by Noé --------------------------------------------------------------
		NearestNeighborsFLANN(unsigned int params_type)
            : index_(0), searchParams_(32, 0., true), dimension_(1)
        {
			params_type_ = params_type;
			/*switch(params_type_) {
				case 1:
					boost::shared_ptr<flann::LinearIndexParams> p;
					params_ = p;
					break;
				case 2:
					boost::shared_ptr<flann::KMeansIndexParams> p;
					params_ = p;
					break;
				case 3:
					boost::shared_ptr<flann::KDTreeIndexParams> p;
					params_ = p;
					break;
				case 4:
					boost::shared_ptr<flann::KDTreeSingleIndexParams> p;
					params_ = p;
					break;
				case 5:
					boost::shared_ptr<flann::CompositeIndexParams> p;
					params_ = p;
					break;
				default:
					boost::shared_ptr<flann::LinearIndexParams> p;
					params_ = p;
			}*/
        }
		//-----------------------------------------------------------------------------

        NearestNeighborsFLANN(const boost::shared_ptr<flann::IndexParams> &params)
            : index_(0), params_(params), searchParams_(32, 0., true), dimension_(1)
        {
        }

        virtual ~NearestNeighborsFLANN(void)
        {
            if (index_)
                delete index_;
        }

        virtual void clear(void)
        {
            if (index_)
            {
                delete index_;
                index_ = NULL;
            }
            //printf("Clearing neirestNeighbors!!!!\n");
            data_.clear();
        }
        
        
        virtual void clearData(void)
        {
            /*if (index_)
            {
                delete index_;
                index_ = NULL;
            }*/
            data_.clear();
        }
        
        

        virtual void setDistanceFunction(const typename NearestNeighbors<_T>::DistanceFunction &distFun)
        {
            NearestNeighbors<_T>::setDistanceFunction(distFun);
            rebuildIndex();
        }

        virtual void add(const _T &data)
        {
            bool rebuild = index_ && (data_.size() + 1 > data_.capacity());

            if (rebuild)
                rebuildIndex(2 * data_.capacity());

            data_.push_back(data);
            const flann::Matrix<_T> mat(&data_.back(), 1, dimension_);

            if (index_)
                index_->addPoints(mat, std::numeric_limits<float>::max()/size());
            else
                createIndex(mat);
        }
        virtual void add(const std::vector<_T> &data)
        {
            unsigned int oldSize = data_.size();
            unsigned int newSize = oldSize + data.size();
            bool rebuild = index_ && (newSize > data_.capacity());

            if (rebuild)
                rebuildIndex(std::max(2 * oldSize, newSize));

            if (index_)
            {
                std::copy(data.begin(), data.end(), data_.begin() + oldSize);
                const flann::Matrix<_T> mat(&data_[oldSize], data.size(), dimension_);
                index_->addPoints(mat, std::numeric_limits<float>::max()/size());
            }
            else
            {
                data_ = data;
                const flann::Matrix<_T> mat(&data_[0], data_.size(), dimension_);
                createIndex(mat);
            }
        }
        virtual bool remove(const _T& data)
        {
            if (!index_) return false;
            _T& elt = const_cast<_T&>(data);
            const flann::Matrix<_T> query(&elt, 1, dimension_);
            std::vector<std::vector<size_t> > indices(1);
            std::vector<std::vector<double> > dists(1);
            index_->knnSearch(query, indices, dists, 1, searchParams_);
            if (*index_->getPoint(indices[0][0]) == data)
            {
                index_->removePoint(indices[0][0]);
                rebuildIndex();
                return true;
            }
            return false;
        }
        virtual _T nearest(const _T &data) const
        {
            if (size())
            {
                _T& elt = const_cast<_T&>(data);
                const flann::Matrix<_T> query(&elt, 1, dimension_);
                std::vector<std::vector<size_t> > indices(1);
                std::vector<std::vector<double> > dists(1);
                index_->knnSearch(query, indices, dists, 1, searchParams_);
                return *index_->getPoint(indices[0][0]);
            }
            //throw Exception("No elements found in nearest neighbors data structure");
			throw std::runtime_error(std::string("No elements found in nearest neighbors data structure"));
        }
        
        
        virtual void nearestK(const _T &data, std::size_t k, std::vector<_T> &nbh) const
        {
            _T& elt = const_cast<_T&>(data);
            const flann::Matrix<_T> query(&elt, 1, dimension_);
            std::vector<std::vector<size_t> > indices;
            std::vector<std::vector<double> > dists;
            k = index_ ? index_->knnSearch(query, indices, dists, k, searchParams_) : 0;
            nbh.resize(k);
            for (std::size_t i = 0 ; i < k ; ++i)
                nbh[i] = *index_->getPoint(indices[0][i]);
        }


        virtual void nearestR(const _T &data, double radius, std::vector<_T> &nbh) const
        {
            _T& elt = const_cast<_T&>(data);
            flann::Matrix<_T> query(&elt, 1, dimension_);
            std::vector<std::vector<size_t> > indices;
            std::vector<std::vector<double> > dists;
            int k = index_ ? index_->radiusSearch(query, indices, dists, radius, searchParams_) : 0;
            nbh.resize(k);
            for (int i = 0 ; i < k ; ++i)
                nbh[i] = *index_->getPoint(indices[0][i]);
        }

        virtual std::size_t size(void) const
        {
            return index_ ? index_->size() : 0;
        }


        virtual void list(std::vector<_T> &data) const
        {
            std::size_t sz = size();
            if (sz == 0)
            {
                data.resize(0);
                return;
            }
            const _T& dummy = *index_->getPoint(0);
            int checks = searchParams_.checks;
            searchParams_.checks = size();
            nearestK(dummy, sz, data);
            searchParams_.checks = checks;
        }
        
        //---added by Noé------------
        void getTree(std::vector<_T> &data) const
        {
            std::size_t sz = size();
            if (sz == 0)
            {
                data.resize(0);
                return;
            }
            data.resize(sz);
            for(int i = 0 ; i < ((int)sz -1) ; i++) {
                data[i] = *index_->getPoint(i);
                //memcpy(&data[i], index_->getPoint(i), sizeof(*index_->getPoint(i)));
			 }
        }
        //----------------------------

        /// \brief Set the FLANN index parameters.
        ///
        /// The parameters determine the type of nearest neighbor
        /// data structure to be constructed.
        /*virtual void setIndexParams(const boost::shared_ptr<flann::IndexParams> &params)
        {
            params_ = params;
            rebuildIndex();
        }

        /// \brief Get the FLANN parameters used to build the current index.
        virtual const boost::shared_ptr<flann::IndexParams>& getIndexParams(void) const
        {
            return params_;
        }*/

        /// \brief Set the FLANN parameters to be used during nearest neighbor
        /// searches
        virtual void setSearchParams(const flann::SearchParams& searchParams)
        {
            searchParams_ = searchParams;
        }

        /// \brief Get the FLANN parameters used during nearest neighbor
        /// searches
        flann::SearchParams& getSearchParams(void)
        {
            return searchParams_;
        }

        /// \brief Get the FLANN parameters used during nearest neighbor
        /// searches
        const flann::SearchParams& getSearchParams(void) const
        {
            return searchParams_;
        }

        unsigned int getContainerSize(void) const
        {
            return dimension_;
        }
        
        
        

    protected:

        /// \brief Internal function to construct nearest neighbor
        /// data structure with initial elements stored in mat.
        void createIndex(const flann::Matrix<_T>& mat)
        {
			//---Modified by Noé
            //index_ = new flann::Index<_Dist>(mat, *params_, _Dist(NearestNeighbors<_T>::distFun_));
			switch(params_type_) {
				case 1:
					index_ = new flann::Index<_Dist>(mat, flann::LinearIndexParams(), _Dist(NearestNeighbors<_T>::distFun_));
					break;
				case 2:
					index_ = new flann::Index<_Dist>(mat, flann::KMeansIndexParams(), _Dist(NearestNeighbors<_T>::distFun_));
					break;
				case 3:
					index_ = new flann::Index<_Dist>(mat, flann::KDTreeIndexParams(), _Dist(NearestNeighbors<_T>::distFun_));
					break;
				case 4:
					index_ = new flann::Index<_Dist>(mat, flann::KDTreeSingleIndexParams(), _Dist(NearestNeighbors<_T>::distFun_));
					break;
				case 5:
					index_ = new flann::Index<_Dist>(mat, flann::CompositeIndexParams(), _Dist(NearestNeighbors<_T>::distFun_));
					break;
				default:
					index_ = new flann::Index<_Dist>(mat, flann::LinearIndexParams(), _Dist(NearestNeighbors<_T>::distFun_));
			}
            index_->buildIndex();
        }

        /// \brief Rebuild the nearest neighbor data structure (necessary when
        /// changing the distance function or index parameters).
        void rebuildIndex(unsigned int capacity = 0)
        {
            if (index_)
            {
                std::vector<_T> data;
                list(data);
                clear();
                if (capacity)
                    data_.reserve(capacity);
                add(data);
            }
        }

        /// \brief vector of data stored in FLANN's index. FLANN only indexes
        /// references, so we need store the original data.
        std::vector<_T>                       data_;

        /// \brief The FLANN index (the actual index type depends on params_).
        flann::Index<_Dist>*                  index_;

        /// \brief The FLANN index parameters. This contains both the type of
        /// index and the parameters for that type.
        boost::shared_ptr<flann::IndexParams> params_;

        /// \brief The parameters used to seach for nearest neighbors.
        mutable flann::SearchParams           searchParams_;

        /// \brief If each element has an array-like structure that is exposed
        /// to FLANN, then the dimension_ needs to be set to the length of
        /// this array.
        unsigned int                          dimension_;
        
        //--added by Noé----
        /// \brief Type of index
        unsigned int						  params_type_;
        
    };

    
	/*template<>
    void NearestNeighborsFLANN<double, flann::L2<double> >::createIndex(
        const flann::Matrix<double>& mat)
    {
        index_ = new flann::Index<flann::L2<double> >(mat, *params_);
        index_->buildIndex();
    }*/

    template<typename _T, typename _Dist = FLANNDistance<_T> >
    class NearestNeighborsFLANNLinear : public NearestNeighborsFLANN<_T, _Dist>
    {
    public:
        NearestNeighborsFLANNLinear()
            : NearestNeighborsFLANN<_T, _Dist>(
                boost::shared_ptr<flann::LinearIndexParams>(
                    new flann::LinearIndexParams()))
        {
        }
    };

    template<typename _T, typename _Dist = FLANNDistance<_T> >
    class NearestNeighborsFLANNHierarchicalClustering : public NearestNeighborsFLANN<_T, _Dist>
    {
    public:
        NearestNeighborsFLANNHierarchicalClustering()
            : NearestNeighborsFLANN<_T, _Dist>(
                boost::shared_ptr<flann::HierarchicalClusteringIndexParams>(
                    new flann::HierarchicalClusteringIndexParams()))
        {
        }
    };

#endif
