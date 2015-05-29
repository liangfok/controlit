#include <controlit/parser/parse_list.hpp>

// using namespace drc::common;
using namespace YAML;

namespace YAML
{
    void operator>>(const Node &in, std::vector<std::string> &x)
    {
        unsigned int rows = in.size();
        x = std::vector<std::string>(rows,"");
        for (unsigned int i = 0; i < rows; ++i)
        {
            in[i] >> x[i];
        }
    }

    Emitter &operator<<(Emitter &out, const std::vector<std::string> &x)
    {
        out << Flow << BeginSeq;
        for (unsigned int i = 0; i < x.size(); ++i)
        {
            out << x[i];
        }
        out << EndSeq;
        return out;
    }
}
