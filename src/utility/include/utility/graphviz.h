/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_GRAPHVIZ_H__
#define __CAUV_GRAPHVIZ_H__

#include <string>

#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/operators.hpp>


namespace graphviz {

#include <graphviz/types.h>
#include <graphviz/gvc.h>
#include <graphviz/cgraph.h>

class Graph;
class Node
{
    public:
        std::string name() const
        {
            return agnameof(const_cast<Agnode_t*>(m_node));
        }
        // NB: returned values are in pt (=1/72 inch)
        pointf coord() const
        {
            return ND_coord(const_cast<Agnode_t*>(m_node));
        }

        template<typename T>
        void attr(const std::string& name, T val)
        {
            agset(m_node, const_cast<char*>(name.c_str()), const_cast<char*>(boost::lexical_cast<std::string>(val).c_str()));
        }

        std::string attr(const std::string& name) const
        {
            char* c = agget(const_cast<Agnode_t*>(m_node), const_cast<char*>(name.c_str()));
            return c ? c : "";
        }

    protected:
        Node(Agnode_t* node) : m_node(node) {}
        Agnode_t* m_node;
        friend class Graph;
};
class Edge
{
    public:

    protected:
        Edge(Agedge_t* edge) : m_edge(edge) {}
        Agedge_t* m_edge;
        friend class Graph;
};


void AgraphDeleter(Agraph_t* v) { agclose(v); }     
class Graph
{
    public:
        Graph(const std::string& name, int type)
            : nodes(this),
              m_G( boost::shared_ptr<Agraph_t>( agopen(const_cast<char*>(name.c_str()), type), AgraphDeleter ) )
        {
        }

        Node node(const std::string& name)
        {
            return Node(agnode(m_G.get(), const_cast<char*>(name.c_str())));
        }
        Edge edge(Node& n1, Node& n2)
        {
            return Edge(agedge(m_G.get(), n1.m_node, n2.m_node));
        }
        Graph subGraph(const std::string& name)
        {
            return Graph(agsubg(m_G.get(), const_cast<char*>(name.c_str())), m_root ? m_root : boost::shared_ptr<Graph>(this)); 
        }

        Agraph_t* get()
        {
            return m_G.get();
        }

        template<typename T>
        void addGraphAttr(const std::string& name, T defaultval)
        {
            agraphattr(m_G.get(), const_cast<char*>(name.c_str()), const_cast<char*>(boost::lexical_cast<std::string>(defaultval).c_str()));
        }
        template<typename T>
        void addNodeAttr(const std::string& name, T defaultval)
        {
            agnodeattr(m_G.get(), const_cast<char*>(name.c_str()), const_cast<char*>(boost::lexical_cast<std::string>(defaultval).c_str()));
        }


        class NodeList
        {
            public:
                NodeList(Graph* graph) : m_graph(graph)
                {
                }
                struct NodeListIterator : public boost::forward_iterator_helper<NodeListIterator, Node, std::ptrdiff_t, Node*, Node&>
                {
                    public:
                        Node operator*() const { return Node(m_node); }
                        void operator++() { m_node = agnxtnode(m_graph.get(), m_node); }
                        bool operator==(const NodeListIterator& i) const { return m_node == i.m_node; }

                    protected:
                        NodeListIterator(boost::shared_ptr<Agraph_t> graph, Agnode_t* node) : m_graph(graph), m_node(node) { }

                        boost::shared_ptr<Agraph_t> m_graph;
                        Agnode_t* m_node;
                
                    friend class NodeList;
                };
                typedef NodeListIterator iterator;
                typedef NodeListIterator const_iterator;

                iterator begin() const { return iterator(m_graph->m_G, agfstnode(m_graph->m_G.get())); }
                iterator end() const { return iterator(m_graph->m_G, NULL); }

            protected:
                Graph* m_graph;
        };

        NodeList nodes;
        //EdgeList edges;

    protected:
        Graph(Agraph_t* G, boost::shared_ptr<Graph> root) : nodes(this), m_G(boost::shared_ptr<Agraph_t>(G, AgraphDeleter)), m_root(root) {}
        
        boost::shared_ptr<Agraph_t> m_G;
        boost::shared_ptr<Graph> m_root;
};

void ContextDeleter(GVC_t* v) { gvFreeContext(v); }     
class Context
{
    public:
        Context()
        {
            m_GVC = boost::shared_ptr<GVC_t>(gvContext(), ContextDeleter);
        }
        
        GVC_t* get() { return m_GVC.get(); }


    protected:
        boost::shared_ptr<GVC_t> m_GVC;
};


}

#endif//__CAUV_GRAPHVIZ_H__
