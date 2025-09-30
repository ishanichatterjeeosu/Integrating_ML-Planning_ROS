/* \author Benjamin Cohen */

#ifndef _LEATHERMAN_UTILS_
#define _LEATHERMAN_UTILS_

// standard includes
#include <string>
#include <vector>

// system includes
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/MultiDOFJointState.h>
#include <std_msgs/ColorRGBA.h>

namespace leatherman {

////////////////////////
// Geometry Utilities //
////////////////////////

double distanceBetween3DLineSegments(
    const Eigen::Vector3d& l1a, const Eigen::Vector3d& l1b,
    const Eigen::Vector3d& l2a, const Eigen::Vector3d& l2b);

///////////////////////////
// Joint State Utilities //
///////////////////////////

bool isValidJointState(const sensor_msgs::JointState& state);
bool isValidMultiDOFJointState(const sensor_msgs::MultiDOFJointState& state);

bool findJointPosition(
    const sensor_msgs::JointState& state,
    std::string name,
    double& position);

/// \brief Extract a subset of joints from a joint state message.
bool getJointPositions(
    const sensor_msgs::JointState& joint_state,
    const sensor_msgs::MultiDOFJointState& multi_dof_joint_state,
    const std::vector<std::string>& joint_names,
    std::vector<double>& positions);

/// \brief Extract a subset of joints from a joint state message and report any
///     missing joints.
bool getJointPositions(
    const sensor_msgs::JointState& joint_state,
    const sensor_msgs::MultiDOFJointState& multi_dof_joint_state,
    const std::vector<std::string>& joint_names,
    std::vector<double>& positions,
    std::vector<std::string>& missing);

void findAndReplaceJointPosition(
    std::string name,
    double position,
    sensor_msgs::JointState& state);

///////////////////
// KDL Utilities //
///////////////////

bool getJointIndex(const KDL::Chain& c, std::string name, int& index);

bool getSegmentIndex(const KDL::Chain& c, std::string name, int& index);

bool getSegmentOfJoint(
    const KDL::Tree& tree,
    std::string joint,
    std::string& segment);

bool getChainTip(
    const KDL::Tree& tree,
    const std::vector<std::string>& segments,
    std::string chain_root, std::string& chain_tip);

////////////
// Colors //
////////////

/// \brief Convert an (H, S, V) color triplet to (R, G, B) format.
/// \param h The hue in range [0, 360)
/// \param s The saturation in range [0, 1]
/// \param v The value in range [0, 1]
void HSVtoRGB(double* r, double* g, double* b, double h, double s, double v);
void msgRGBToHSV(const std_msgs::ColorRGBA& color, double& h, double& s, double& v);
void msgHSVToRGB(double h, double s, double v, std_msgs::ColorRGBA& color);

inline std_msgs::ColorRGBA MakeColorRGBA(float r, float g, float b, float a = 1.0f)
{
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

inline std_msgs::ColorRGBA MakeColorARGB(uint32_t val)
{
    return MakeColorRGBA(
            (float)(val << 8  >> 24) / 255.0f,
            (float)(val << 16 >> 24) / 255.0f,
            (float)(val << 24 >> 24) / 255.0f,
            (float)(val << 0  >> 24) / 255.0f);
}

namespace colors {

// 140 Named Colors Commonly Supported by Browsers (https://www.w3schools.com/colors/colors_names.asp)
inline auto AliceBlue() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFF0F8FF); }
inline auto AntiqueWhite() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFAEBD7); }
inline auto Aqua() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF00FFFF); }
inline auto Aquamarine() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF7FFFD4); }
inline auto Azure() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFF0FFFF); }
inline auto Beige() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFF5F5DC); }
inline auto Bisque() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFFE4C4); }
inline auto Black() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF000000); }
inline auto BlanchedAlmond() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFFEBCD); }
inline auto Blue() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF0000FF); }
inline auto BlueViolet() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF8A2BE2); }
inline auto Brown() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFA52A2A); }
inline auto BurlyWood() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFDEB887); }
inline auto CadetBlue() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF5F9EA0); }
inline auto Chartreuse() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF7FFF00); }
inline auto Chocolate() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFD2691E); }
inline auto Coral() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFF7F50); }
inline auto CornflowerBlue() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF6495ED); }
inline auto Cornsilk() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFFF8DC); }
inline auto Crimson() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFDC143C); }
inline auto Cyan() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF00FFFF); }
inline auto DarkBlue() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF00008B); }
inline auto DarkCyan() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF008B8B); }
inline auto DarkGoldenRod() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFB8860B); }
inline auto DarkGray() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFA9A9A9); }
inline auto DarkGrey() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFA9A9A9); }
inline auto DarkGreen() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF006400); }
inline auto DarkKhaki() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFBDB76B); }
inline auto DarkMagenta() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF8B008B); }
inline auto DarkOliveGreen() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF556B2F); }
inline auto DarkOrange() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFF8C00); }
inline auto DarkOrchid() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF9932CC); }
inline auto DarkRed() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF8B0000); }
inline auto DarkSalmon() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFE9967A); }
inline auto DarkSeaGreen() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF8FBC8F); }
inline auto DarkSlateBlue() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF483D8B); }
inline auto DarkSlateGray() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF2F4F4F); }
inline auto DarkSlateGrey() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF2F4F4F); }
inline auto DarkTurquoise() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF00CED1); }
inline auto DarkViolet() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF9400D3); }
inline auto DeepPink() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFF1493); }
inline auto DeepSkyBlue() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF00BFFF); }
inline auto DimGray() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF696969); }
inline auto DimGrey() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF696969); }
inline auto DodgerBlue() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF1E90FF); }
inline auto FireBrick() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFB22222); }
inline auto FloralWhite() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFFFAF0); }
inline auto ForestGreen() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF228B22); }
inline auto Fuchsia() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFF00FF); }
inline auto Gainsboro() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFDCDCDC); }
inline auto GhostWhite() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFF8F8FF); }
inline auto Gold() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFFD700); }
inline auto GoldenRod() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFDAA520); }
inline auto Gray() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF808080); }
inline auto Grey() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF808080); }
inline auto Green() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF008000); }
inline auto GreenYellow() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFADFF2F); }
inline auto HoneyDew() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFF0FFF0); }
inline auto HotPink() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFF69B4); }
inline auto IndianRed() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFCD5C5C); }
inline auto Indigo() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF4B0082); }
inline auto Ivory() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFFFFF0); }
inline auto Khaki() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFF0E68C); }
inline auto Lavender() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFE6E6FA); }
inline auto LavenderBlush() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFFF0F5); }
inline auto LawnGreen() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF7CFC00); }
inline auto LemonChiffon() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFFFACD); }
inline auto LightBlue() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFADD8E6); }
inline auto LightCoral() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFF08080); }
inline auto LightCyan() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFE0FFFF); }
inline auto LightGoldenRodYellow() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFAFAD2); }
inline auto LightGray() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFD3D3D3); }
inline auto LightGrey() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFD3D3D3); }
inline auto LightGreen() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF90EE90); }
inline auto LightPink() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFFB6C1); }
inline auto LightSalmon() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFFA07A); }
inline auto LightSeaGreen() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF20B2AA); }
inline auto LightSkyBlue() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF87CEFA); }
inline auto LightSlateGray() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF778899); }
inline auto LightSlateGrey() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF778899); }
inline auto LightSteelBlue() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFB0C4DE); }
inline auto LightYellow() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFFFFE0); }
inline auto Lime() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF00FF00); }
inline auto LimeGreen() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF32CD32); }
inline auto Linen() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFAF0E6); }
inline auto Magenta() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFF00FF); }
inline auto Maroon() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF800000); }
inline auto MediumAquaMarine() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF66CDAA); }
inline auto MediumBlue() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF0000CD); }
inline auto MediumOrchid() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFBA55D3); }
inline auto MediumPurple() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF9370DB); }
inline auto MediumSeaGreen() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF3CB371); }
inline auto MediumSlateBlue() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF7B68EE); }
inline auto MediumSpringGreen() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF00FA9A); }
inline auto MediumTurquoise() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF48D1CC); }
inline auto MediumVioletRed() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFC71585); }
inline auto MidnightBlue() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF191970); }
inline auto MintCream() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFF5FFFA); }
inline auto MistyRose() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFFE4E1); }
inline auto Moccasin() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFFE4B5); }
inline auto NavajoWhite() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFFDEAD); }
inline auto Navy() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF000080); }
inline auto OldLace() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFDF5E6); }
inline auto Olive() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF808000); }
inline auto OliveDrab() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF6B8E23); }
inline auto Orange() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFFA500); }
inline auto OrangeRed() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFF4500); }
inline auto Orchid() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFDA70D6); }
inline auto PaleGoldenRod() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFEEE8AA); }
inline auto PaleGreen() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF98FB98); }
inline auto PaleTurquoise() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFAFEEEE); }
inline auto PaleVioletRed() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFDB7093); }
inline auto PapayaWhip() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFFEFD5); }
inline auto PeachPuff() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFFDAB9); }
inline auto Peru() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFCD853F); }
inline auto Pink() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFFC0CB); }
inline auto Plum() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFDDA0DD); }
inline auto PowderBlue() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFB0E0E6); }
inline auto Purple() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF800080); }
inline auto RebeccaPurple() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF663399); }
inline auto Red() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFF0000); }
inline auto RosyBrown() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFBC8F8F); }
inline auto RoyalBlue() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF4169E1); }
inline auto SaddleBrown() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF8B4513); }
inline auto Salmon() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFA8072); }
inline auto SandyBrown() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFF4A460); }
inline auto SeaGreen() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF2E8B57); }
inline auto SeaShell() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFFF5EE); }
inline auto Sienna() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFA0522D); }
inline auto Silver() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFC0C0C0); }
inline auto SkyBlue() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF87CEEB); }
inline auto SlateBlue() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF6A5ACD); }
inline auto SlateGray() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF708090); }
inline auto SlateGrey() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF708090); }
inline auto Snow() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFFFAFA); }
inline auto SpringGreen() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF00FF7F); }
inline auto SteelBlue() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF4682B4); }
inline auto Tan() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFD2B48C); }
inline auto Teal() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF008080); }
inline auto Thistle() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFD8BFD8); }
inline auto Tomato() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFF6347); }
inline auto Turquoise() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF40E0D0); }
inline auto Violet() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFEE82EE); }
inline auto Wheat() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFF5DEB3); }
inline auto White() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFFFFFF); }
inline auto WhiteSmoke() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFF5F5F5); }
inline auto Yellow() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFFFFFF00); }
inline auto YellowGreen() -> std_msgs::ColorRGBA { return MakeColorARGB(0xFF9ACD32); }

} // namespace Colors

/////////////////
// ROS Logging //
/////////////////

void setLoggerLevel(std::string package, std::string name, std::string level);
void setLoggerLevel(std::string name, std::string level);

} // namespace leatherman

#endif
