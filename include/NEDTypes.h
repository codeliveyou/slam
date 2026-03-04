/**
* NED (North-East-Down) absolute position constraint types for ORB-SLAM3.
*
* Provides a data structure for matched point pairs between the SLAM map
* and a georeferenced map (NED coordinates), plus a custom g2o edge that
* anchors KeyFrame Sim3 vertices to absolute positions through observed
* MapPoints.
*/

#ifndef NEDTYPES_H
#define NEDTYPES_H

#include <Eigen/Core>
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM3
{

class MapPoint;

struct NEDMatch
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapPoint* pMP;
    Eigen::Vector3d nedPos;

    NEDMatch() : pMP(nullptr), nedPos(Eigen::Vector3d::Zero()) {}
    NEDMatch(MapPoint* mp, const Eigen::Vector3d& ned) : pMP(mp), nedPos(ned) {}
};

} // namespace ORB_SLAM3


namespace g2o
{

/**
 * Unary edge on a VertexSim3Expmap that constrains a pre-projected 3D point
 * to land at a known absolute NED position after applying the corrected Sim3.
 *
 * Vertex 0:  S_iw  (corrected world-to-camera Sim3 for reference KeyFrame i)
 * Measurement:  P_ned  (absolute 3D position in NED frame, Vector3d)
 * Stored data:  _pointInCamera = S_iw_old.map(P_w_old)  (precomputed constant)
 *
 * Error:  e = P_ned - S_wi_corrected.map(_pointInCamera)
 *       where S_wi_corrected = S_iw_corrected^{-1}
 *
 * The Jacobian is computed numerically by g2o's default mechanism.
 */
class EdgeNEDPointConstraint
    : public BaseUnaryEdge<3, Eigen::Vector3d, VertexSim3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeNEDPointConstraint() : _pointInCamera(Eigen::Vector3d::Zero()) {}

    virtual bool read(std::istream& /*is*/) { return true; }
    virtual bool write(std::ostream& /*os*/) const { return true; }

    void computeError()
    {
        const VertexSim3Expmap* v =
            static_cast<const VertexSim3Expmap*>(_vertices[0]);
        Sim3 Swi = v->estimate().inverse();
        Eigen::Vector3d correctedWorldPos = Swi.map(_pointInCamera);
        _error = _measurement - correctedWorldPos;
    }

    Eigen::Vector3d _pointInCamera;
};

} // namespace g2o

#endif // NEDTYPES_H
