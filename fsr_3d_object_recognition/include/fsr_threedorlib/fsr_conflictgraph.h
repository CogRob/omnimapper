#ifndef FSR_CONFLICTGRAPH
#define FSR_CONFLICTGRAPH

#include <tbb/concurrent_vector.h>
#include <boost/unordered_set.hpp>

#include <fsr_threedorlib/fsr_types.h>

namespace fsr_or
{

  typedef std::pair<int, int> CNPoint;

  struct ConflictNode
  {
    OMKey::Ptr id;
    int m_v;
    int m_p;
    boost::unordered_set<CNPoint> explainedPoints;
    bool suppressed;

    ConflictNode () {}

    ConflictNode (OMKey::Ptr &id, int m_v, int m_p, boost::unordered_set<CNPoint> &explainedPoints)
    : id (id),
      m_v (m_v),
      m_p (m_p),
      explainedPoints (explainedPoints),
      suppressed (false)
    {}

    ConflictNode (OMKey::Ptr &id)
    : id (id),
      m_v (0),
      m_p (0),
      explainedPoints (),
      suppressed (false)
    {}
  };

  struct ConflictEdge
  {
    int from;
    int to;
    int conflicts;

    ConflictEdge () {}

    ConflictEdge (int from, int to, int conflicts)
    : from (from),
      to (to),
      conflicts (conflicts)
    {}
  };

  struct ConflictGraph
  {
    tbb::concurrent_vector<ConflictNode> nodes;
    tbb::concurrent_vector<ConflictEdge> edges;

    ConflictGraph ()
    : nodes (tbb::concurrent_vector<ConflictNode> ()),
      edges (tbb::concurrent_vector<ConflictEdge> ())
    {}

    inline void addNode (ConflictNode &node)
    {
      nodes.push_back (node);
    }

    void addEdge (int f, int t)
    {
      int c = 0;
      CNPoint fpt;
      boost::unordered_set<CNPoint> from = nodes[f].explainedPoints;
      boost::unordered_set<CNPoint> to = nodes[t].explainedPoints;
      for (boost::unordered_set<CNPoint>::iterator it = from.begin (); it != from.end (); ++it)
      {
        //c += (to.find (*it) != to.end ()) ? 1 : 0;
        if (to.find (*it) != to.end ())
        {
          ++c;
          break;
        }
      }

      if (c > 0)
      {
        edges.push_back (ConflictEdge (f, t, c));
      }
    }

    inline void suppressNodeOnEdgeFrom (int i)
    {
      ConflictNode from = nodes[edges[i].from];
      ConflictNode to = nodes[edges[i].to];

      /// suppress point if its neighbor explains more points
      if (to.m_p > from.m_p)
      {
        from.suppressed = true;
      }
      /// break ties by picking the one that occludes less points
      else if (to.m_p == from.m_p && to.m_v < from.m_v)
      {
        from.suppressed = true;
      }
    }

    inline void suppressNodeOnEdgeTo (int i)
    {
      ConflictNode from = nodes[edges[i].from];
      ConflictNode to = nodes[edges[i].to];

      if (!to.suppressed)
      {
        if (to.m_p < from.m_p)
        {
          to.suppressed = true;
        }
        else if (to.m_p == from.m_p && to.m_v > from.m_v)
        {
          to.suppressed = true;
        }
      }
    }

    inline bool nodeSuppressed (int i)
    {
      return nodes[i].suppressed;
    }

    inline void getNodeID (int i, OMKey::Ptr &id)
    {
      id = nodes[i].id;
    }

    /// for debugging puposes
    inline ConflictNode& getNodeByID (OMKey::Ptr &id)
    {
      for (tbb::concurrent_vector<ConflictNode>::iterator it = nodes.begin (); it != nodes.end (); ++it)
      {
        if (*(it->id) == *id)
        {
          return *it;
        }
      }
    }

    inline size_t size () { return nodes.size (); }
    inline size_t edgeCapacity () { return edges.size (); }
  };

}

#endif // FSR_RECOGNITION

