#ifndef RRT_NEAREST_NEIGHBORS_
#define RRT_NEAREST_NEIGHBORS_

#include <vector>
#include <boost/bind.hpp>
#include <boost/function.hpp>



    /** \brief Abstract representation of a container that can perform nearest neighbors queries */
    template<typename _T>
    class NearestNeighbors
    {
    public:

        /** \brief The definition of a distance function */
        typedef boost::function<double(const _T&, const _T&)> DistanceFunction;

        NearestNeighbors(void)
        {
        }

        virtual ~NearestNeighbors(void)
        {
        }

        /** \brief Set the distance function to use */
        virtual void setDistanceFunction(const DistanceFunction &distFun)
        {
            distFun_ = distFun;
        }

        /** \brief Get the distance function used */
        const DistanceFunction& getDistanceFunction(void) const
        {
            return distFun_;
        }

        /** \brief Clear the datastructure */
        virtual void clear(void) = 0;
        
        virtual void clearData(void) = 0; //Added by Noé

        /** \brief Add an element to the datastructure */
        virtual void add(const _T &data) = 0;

        /** \brief Add a vector of points */
        virtual void add(const std::vector<_T> &data)
        {
            for (typename std::vector<_T>::const_iterator elt = data.begin() ; elt != data.end() ; ++elt)
                add(*elt);
        }

        /** \brief Remove an element from the datastructure */
        virtual bool remove(const _T &data) = 0;

        /** \brief Get the nearest neighbor of a point */
        virtual _T nearest(const _T &data) const = 0;

        /** \brief Get the k-nearest neighbors of a point */
        virtual void nearestK(const _T &data, std::size_t k, std::vector<_T> &nbh) const = 0;

        /** \brief Get the nearest neighbors of a point, within a specified radius */
        virtual void nearestR(const _T &data, double radius, std::vector<_T> &nbh) const = 0;

        /** \brief Get the number of elements in the datastructure */
        virtual std::size_t size(void) const = 0;

        /** \brief Get all the elements in the datastructure */
        virtual void list(std::vector<_T> &data) const = 0;

    protected:

        /** \brief The used distance function */
        DistanceFunction distFun_;

    };
#endif
