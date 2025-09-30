#include <leatherman/mesh_resource.h>

#include <memory>

// system includes
#include <tinyxml.h>
#include <geometric_shapes/mesh_operations.h>
#include <resource_retriever/retriever.h>
#include <ros/console.h>
#include <urdf/model.h>

namespace leatherman {

void getMeshComponents(
    shapes::Mesh* mesh,
    std::vector<std::uint32_t>& triangles,
    std::vector<Eigen::Vector3d>& vertices)
{
    Eigen::Vector3d v;

    // copy vertices
    ROS_DEBUG("vertexCount: %d    triangleCount: %d", mesh->vertex_count, mesh->triangle_count);
    for (unsigned int i = 0; i < mesh->vertex_count; ++i) {
        v.x() = mesh->vertices[3 * i    ];
        v.y() = mesh->vertices[3 * i + 1];
        v.z() = mesh->vertices[3 * i + 2];
        ROS_DEBUG("[vertex %d] xyz: %0.3f %0.3f %0.3f", int(i), v.x(), v.y(), v.z());
        vertices.push_back(v);
    }

    // copy triangles
    triangles.resize(3 * mesh->triangle_count);
    for (unsigned int i = 0; i < 3 * mesh->triangle_count; ++i) {
        triangles[i] = mesh->triangles[i];
    }
}

void getMeshComponents(
    shapes::Mesh* mesh,
    std::vector<double>& vertex_data,
    std::vector<std::uint32_t>& triangles)
{
    vertex_data.resize(3 * mesh->vertex_count);
    std::copy(
            mesh->vertices,
            mesh->vertices + 3 * mesh->vertex_count,
            begin(vertex_data));

    triangles.resize(3 * mesh->triangle_count);
    std::copy(
            mesh->triangles,
            mesh->triangles + 3 * mesh->triangle_count,
            begin(triangles));
}

bool getMeshComponentsFromResource(
    const std::string& resource,
    const Eigen::Vector3d& scale,
    std::vector<std::uint32_t>& triangles,
    std::vector<Eigen::Vector3d>& vertices)
{
    if (resource.empty()) {
        return false;
    }

    std::unique_ptr<shapes::Mesh> mesh;
    mesh.reset(shapes::createMeshFromResource(resource, scale));
    if (!mesh) {
        ROS_ERROR("Failed to load mesh '%s'", resource.c_str());
        return false;
    }

    getMeshComponents(mesh.get(), triangles, vertices);
    return true;
}

bool getMeshComponentsFromResource(
    const std::string& resource,
    const Eigen::Vector3d& scale,
    std::vector<double>& vertex_data,
    std::vector<std::uint32_t>& triangles)
{
    if (resource.empty()) {
        return false;
    }

    std::unique_ptr<shapes::Mesh> mesh;
    mesh.reset(shapes::createMeshFromResource(resource, scale));
    if (!mesh) {
        ROS_ERROR("Failed to load mesh '%s'", resource.c_str());
        return false;
    }

    getMeshComponents(mesh.get(), vertex_data, triangles);
    return true;
}

double getColladaFileScale(std::string resource)
{
    static std::map<std::string, float> rescale_cache;

    // Try to read unit to meter conversion ratio from mesh. Only valid in Collada XML formats.
    TiXmlDocument xmlDoc;
    float unit_scale(1.0);
    resource_retriever::Retriever retriever;
    resource_retriever::MemoryResource res;
    try {
        res = retriever.get(resource);
    } catch (resource_retriever::Exception& e) {
        ROS_ERROR("%s", e.what());
        return unit_scale;
    }

    if (res.size == 0) {
        return unit_scale;
    }

    // Use the resource retriever to get the data.
    const char * data = reinterpret_cast<const char *>(res.data.get());
    xmlDoc.Parse(data);

    // Find the appropriate element if it exists
    if (!xmlDoc.Error()) {
        TiXmlElement * colladaXml = xmlDoc.FirstChildElement("COLLADA");
        if (colladaXml) {
            TiXmlElement *assetXml = colladaXml->FirstChildElement("asset");
            if (assetXml) {
                TiXmlElement *unitXml = assetXml->FirstChildElement("unit");
                if (unitXml && unitXml->Attribute("meter")) {
                    // Failing to convert leaves unit_scale as the default.
                    if (unitXml->QueryFloatAttribute("meter", &unit_scale) != 0)
                        ROS_WARN_STREAM("getMeshUnitRescale::Failed to convert unit element meter attribute to determine scaling. unit element: " << *unitXml);
                }
            }
        }
    }
    return unit_scale;
}

bool getLinkMesh(
    const std::string& urdf,
    const std::string &name,
    bool collision,
    std::string &mesh_resource,
    geometry_msgs::PoseStamped &pose)
{
    urdf::Model model;
    if (!model.initString(urdf)) {
        ROS_ERROR("Something is wrong with the URDF.");
        return false;
    }

    auto link = boost::make_shared<const urdf::Link> ( *model.getLink(name) );
    if (!link) {
        ROS_ERROR("Failed to find link '%s' in URDF.", name.c_str());
        return false;
    }
    if (!link->collision) {
        ROS_ERROR("Failed to find collision field for link '%s' in URDF.", link->name.c_str());
        return false;
    }
    if (!link->collision->geometry) {
        ROS_ERROR("Failed to find geometry for link '%s' in URDF.", name.c_str());
        return false;
    }

    boost::shared_ptr<const urdf::Geometry> geom;
    if (collision) {
        geom = boost::make_shared<const urdf::Geometry>( *link->visual->geometry );
        pose.pose.position.x = link->visual->origin.position.x;
        pose.pose.position.y = link->visual->origin.position.y;
        pose.pose.position.z = link->visual->origin.position.z;
        link->visual->origin.rotation.getQuaternion(
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w);
    } else {
        geom = boost::make_shared<const urdf::Geometry>( *link->collision->geometry );
        pose.pose.position.x = link->collision->origin.position.x;
        pose.pose.position.y = link->collision->origin.position.y;
        pose.pose.position.z = link->collision->origin.position.z;
        link->collision->origin.rotation.getQuaternion(
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w);
    }

    if (geom->type != urdf::Geometry::MESH) {
        ROS_ERROR("Failed because geometry is not a mesh.");
        return false;
    }
    urdf::Mesh* mesh = (urdf::Mesh*)geom.get();
    mesh_resource = mesh->filename;
    pose.header.frame_id = link->parent_joint->child_link_name;
    return true;
}

} // namespace leatherman
