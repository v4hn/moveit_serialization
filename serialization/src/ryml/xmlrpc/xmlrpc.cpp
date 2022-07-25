#include <moveit_serialization/ryml/std/std.h>
#include <moveit_serialization/ryml/format.h>
#include <moveit_serialization/ryml/xmlrpc/xmlrpc.h>
namespace c4 {
namespace yml {

namespace {
class XmlRpcValueCreator : public XmlRpc::XmlRpcValue
{
public:
    static XmlRpcValueCreator createArray(const std::vector<XmlRpcValue>& values)
    {
        XmlRpcValueCreator ret;
        ret._type = TypeArray;
        ret._value.asArray = new ValueArray(values);

        return ret;
    }

    static XmlRpcValueCreator createStruct(const std::map<std::string, XmlRpcValue>& members)
    {
        XmlRpcValueCreator ret;
        ret._type = TypeStruct;
        ret._value.asStruct = new std::map<std::string, XmlRpcValue>(members);
        return ret;
    }
};

void encodeXmlRpc(const XmlRpc::XmlRpcValue& xml_node, NodeRef& node)
{
    switch (xml_node.getType()) {
        case XmlRpc::XmlRpcValue::TypeStruct: {
            node |= c4::yml::MAP;
            for (XmlRpc::XmlRpcValue::const_iterator it = xml_node.begin(); it != xml_node.end(); ++it) {
                auto child = node.append_child();
                child << key(it->first);

                encodeXmlRpc(it->second, child);
            }

            return;
        }
        case XmlRpc::XmlRpcValue::TypeArray: {
            node |= c4::yml::SEQ;
            for (std::size_t i = 0; i < xml_node.size(); ++i) {
                auto child = node.append_child();
                encodeXmlRpc(xml_node[i], child);
            }

            return;
        }
        case XmlRpc::XmlRpcValue::TypeInt: {
            node << static_cast<int>(xml_node);
            return;
        }
        case XmlRpc::XmlRpcValue::TypeBoolean: {
            node << static_cast<bool>(xml_node);
            return;
        }
        case XmlRpc::XmlRpcValue::TypeDouble: {
            node << freal(static_cast<double>(xml_node));
            return;
        }
        case XmlRpc::XmlRpcValue::TypeString: {
            node << static_cast<std::string>(xml_node);
            return;
        }

        default:
            throw std::runtime_error("Unknown node type in XmlRpcValue");
    }
}

// Decode XmlRpcValue primitive
XmlRpc::XmlRpcValue decodeXmlRpcValue(const csubstr& substr)
{
    {  // bool
        bool value;
        if (from_chars(substr, &value))
            return XmlRpc::XmlRpcValue(value);
    }

    {  // int
        int value;
        if (from_chars(substr, &value))
            return XmlRpc::XmlRpcValue(value);
    }

    {  // double
        double value;
        if (from_chars(substr, &value))
            return XmlRpc::XmlRpcValue(value);
    }

    {  // string
        std::string value;
        if (from_chars(substr, &value))
            return XmlRpc::XmlRpcValue(value);
    }

    throw std::runtime_error("Cannot convert ryml::NodeRef to XmlRpc");
}

XmlRpc::XmlRpcValue decodeXmlRpcValue(NodeRef const& node)
{
    if (node.is_container()) {
        if (node.is_map()) {
            std::map<std::string, XmlRpc::XmlRpcValue> members;

            for (NodeRef const& child : node.children()) {
                std::string key;
                from_chars(child.key(), &key);

                members[key] = decodeXmlRpcValue(child);
            }

            return XmlRpcValueCreator::createStruct(members);
        } else if (node.is_seq()) {
            std::vector<XmlRpc::XmlRpcValue> values;

            for (NodeRef const& child : node.children())
                values.push_back(decodeXmlRpcValue(child));

            return XmlRpcValueCreator::createArray(values);
        }
    } else if (node.is_keyval()) {
        // key already stored
        return decodeXmlRpcValue(node.val());
    }

    else if (node.is_val()) {
        return decodeXmlRpcValue(node.val());
    }

    throw std::runtime_error("Cannot convert ryml::NodeRef to XmlRpc");
}
}  // namespace

void write(c4::yml::NodeRef* n, XmlRpc::XmlRpcValue const& rhs)
{
    encodeXmlRpc(rhs, *n);
}

bool read(c4::yml::NodeRef const& n, XmlRpc::XmlRpcValue* rhs)
{
    *rhs = decodeXmlRpcValue(n);

    return true;
}

}  // namespace yml
}  // namespace c4
