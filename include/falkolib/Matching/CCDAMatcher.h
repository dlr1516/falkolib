/**
 * FALKOLib - Fast Adaptive Laser Keypoint Orientation-invariant
 * Copyright (C) 2016 Fabjan Kallasi and Dario Lodi Rizzini.
 *
 * This file is part of FALKOLib.
 *
 * FALKOLib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * FALKOLib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with FALKOLib.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <falkolib/Matching/Matcher.h>
#include <mcqd/mcqd.h>

namespace falkolib {

    /** @brief This class implements the Combined Constraint Data Association (CCDA)
     * proposed by Tim Bailey. The best reference is:
     *
     * T. Bailey, "Mobile Robot Localisation and Mapping in Extensive Outdoor Environments", 
     *  PhD Thesis, University of Sydney, 2002.
     *
     * Of course, there are the related papers. 
     * The method is based on the so called CorrespondenceGraph built on two point sets.
     * Data Association is computed as the maximum clique CorrespondenceGraph. 
     *
     * Maximum clique has been computed according to T. Bailey suggestions.
     * However, for different approaches see:
     *  http://www.dharwadker.org/clique/
     *  http://www.sanfoundry.com/cpp-program-find-maximum-size-clique-graph/
     *  http://www.sicmm.org/~konc/maxclique/mcqd/mcqd.h
     */
    template <typename T = Keypoint, typename D = Descriptor>
    class CCDAMatcher : public Matcher<T> {
    public:

        /** Node of correspondence graph represents an association 
         * between an input point and a target point. 
         */
        struct Node {
            int index; // global index (that should correspond to position in nodes vector)
            int inputId; // input point ID
            int targetId; // target point ID
            std::vector<int> adjacents;

            bool operator<(const Node& n) const {
                return (index < n.index);
            }

            int degree() const {
                return adjacents.size();
            }
        };

        /** Distance constraint between the point belonging to the same set.
         */
        struct Constraint {
            int i;
            int j;
            double dist;

            bool operator<(const Constraint& c) const {
                return (dist < c.dist);
            }
        };

        /** @brief Default constructor. */
        CCDAMatcher() : distTol(0.10), distMin(0.0) {
        }

        /** @brief Sets the tolerance on distances differences to be compatible. See CCDA details. */
        void setDistTol(double _distTol) {
            distTol = _distTol;
        }

        /** @brief Sets the minimum distance between two keypoints to be a constraint. */
        void setDistMin(double _distMin) {
            distMin = _distMin;
        }

        /** Computes the value 
         */
        int match(const std::vector<T>& v1, const std::vector<T>& v2, std::vector<std::pair<int, int> >& match) {
            std::vector<Node> nodes;
            std::vector<std::pair<int, int> > edges;
            std::vector<Constraint> constraints1;
            std::vector<Constraint> constraints2;
            Constraint constrTmp;
            std::vector<int> cliqueMax;
            int num1 = v1.size();
            int num2 = v2.size();
            int isrc, idst;

            match.clear();
            // Creates constraints inside each group of keypoints.
            // A constraint is the geometric distance (must be > distMin!) between two internal keypoints of a given set.
            makeRelativeConstraints(v1, constraints1);
            makeRelativeConstraints(v2, constraints2);

            // Creates correspondence graph based on constraints:
            // * node: association hypothesis n=(k1,k2) where k1 in set1, k2 in set2;
            // * edge: connects 2 compatible association hypotheses (compatibility (na,nb) if ||na.k1-na.k2| - |nb.k1-nb.k2|| < distToll);

            // Nodes
            makeNodeSet(v1, v2, nodes);

            // Edges
            for (auto& constrCurr : constraints1) {
                // Finds the candidate target constraints, i.e. pair of target points with similar 
                // distance, and visits them 
                constrTmp.dist = constrCurr.dist - distTol;
                typename std::vector<Constraint>::iterator it = std::upper_bound(constraints2.begin(), constraints2.end(), constrTmp);
                for (; it != constraints2.end() && it->dist < constrCurr.dist + distTol; ++it) {
                    //std::cout << "  target constr (" << it->i << "," << it->j << "): " << it->dist << std::endl;
                    if (std::abs(it->dist - constrCurr.dist) < distTol && (it->dist + constrCurr.dist) > distMin) {
                        assert(constrCurr.i < num1 && constrCurr.j < num1);
                        assert(it->i < num2 && it->j < num2);
                        // Match 1
                        isrc = it->i + constrCurr.i * num2;
                        idst = it->j + constrCurr.j * num2;
                        assert(isrc < nodes.size() && idst < nodes.size());
                        assert(nodes[isrc].inputId == constrCurr.i && nodes[isrc].targetId == it->i);
                        assert(nodes[idst].inputId == constrCurr.j && nodes[idst].targetId == it->j);
                        nodes[isrc].adjacents.push_back(idst);
                        nodes[idst].adjacents.push_back(isrc);
                        // Match 2
                        isrc = it->i + constrCurr.j * num2;
                        idst = it->j + constrCurr.i * num2;
                        assert(isrc < nodes.size() && idst < nodes.size());
                        assert(nodes[isrc].inputId == constrCurr.j && nodes[isrc].targetId == it->i);
                        assert(nodes[idst].inputId == constrCurr.i && nodes[idst].targetId == it->j);
                        nodes[isrc].adjacents.push_back(idst);
                        nodes[idst].adjacents.push_back(isrc);
                    }
                }
            }

            // Finds maximum clique
            findCliqueDyn(nodes, cliqueMax);
            match.reserve(cliqueMax.size());
            for (auto& id : cliqueMax) {
                match.push_back(std::make_pair(nodes[id].inputId, nodes[id].targetId));
            }
        }

    private:
        double distTol;
        double distMin;

        void makeNodeSet(const std::vector<T>& points1, const std::vector<T>& points2, std::vector<Node>& nodes) {
            Node corresp;
            int index;
            nodes.clear();
            nodes.resize(points1.size() * points2.size());
            for (int i = 0; i < points1.size(); ++i) {
                for (int j = 0; j < points2.size(); ++j) {
                    index = i * points2.size() + j;
                    nodes[index].inputId = i;
                    nodes[index].targetId = j;
                    nodes[index].index = index;
                    nodes[index].adjacents.clear();
                }
            }
            //      for (int i = 0; i < nodes.size(); ++i) {
            //        if (nodes[i].index != i) {
            //          std::cerr << __PRETTY_FUNCTION__ << ": difference bewteen node index " << nodes[i].index << " and its position " << i << " in node vector" << std::endl;
            //        }
            //      }
        }

        void makeRelativeConstraints(const std::vector<T>& points, std::vector<Constraint>& constraints) {
            Constraint constraint;
            int n = points.size();
            for (int i = 0; i < n; ++i) {
                for (int j = i + 1; j < n; ++j) {
                    constraint.i = i;
                    constraint.j = j;
                    constraint.dist = points[i].distance(points[j]);
                    constraints.push_back(constraint);
                }
            }
            std::sort(constraints.begin(), constraints.end());
        }

        void findCliqueDyn(const std::vector<Node>& nodes, std::vector<int>& cliqueMax) {
            bool **conn;
            int nsize = nodes.size();
            int *qmax;
            int qsize;

            if (nsize == 0) {
                return;
            }

            // Allocates space for adjacence matrix 
            conn = new bool*[nsize];
            for (int i = 0; i < nsize; ++i) {
                conn[i] = new bool[nsize];
                memset(conn[i], 0, nsize * sizeof (bool));
            }
            // Fills adjacence matrix
            for (int i = 0; i < nodes.size(); ++i) {
                assert(nodes[i].index == i);
                for (auto& j : nodes[i].adjacents) {
                    conn[i][j] = true;
                }
            }
            // Computes maximum clique
            Maxclique maxcl(conn, nsize);
            maxcl.mcq(qmax, qsize);
            cliqueMax.resize(qsize);
            for (int i = 0; i < qsize; ++i) {
                cliqueMax[i] = qmax[i];
            }

            delete [] qmax;
            for (int i = 0; i < nsize; ++i)
                delete [] conn[i];
            delete [] conn;
        }

    };
}
