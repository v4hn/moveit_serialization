#include <moveit_serialization/ryml/utils.h>
#include <moveit_serialization/ryml/ryml.h>
#include <iostream>
/* Given */
bool c4::yml::getNodeFromKeyChainVal(const c4::yml::NodeRef& source, const c4::yml::NodeRef& target,
                                     c4::yml::NodeRef& scalar)
{
    // std::cout << "=== source ===" << std::endl;
    // std::cout << "type = " << source.type_str() << std::endl;
    // std::cout << source << std::endl;

    // std::cout << "=== target ===" << std::endl;
    // std::cout << "type = " << target.type_str() << std::endl;
    // std::cout << target << std::endl;

    if (not(source.type() == c4::yml::VAL || source.type() == c4::yml::KEYVAL))
        if (source.type() != target.type())
            return false;

    if (source.type() == c4::yml::VAL || source.type() == c4::yml::KEYVAL) {
        if (!target.has_child(source.val()))
            return false;

        // clone value
        scalar.tree()->merge_with(target.tree(), target.find_child(source.val()).id(), ryml::NONE);
        return true;

    } else if (source.type() == c4::yml::NONE)
        return false;
    else if (source.type() == c4::yml::MAP || source.type() == c4::yml::SEQ) {
        if (source.num_children() > target.num_children())
            return false;
    }

    // loop recursively through map
    if (source.is_map()) {
        bool result = true;

        for (c4::yml::NodeRef const& child : source.children()) {
            if (target.has_child(child.key()))
                result = getNodeFromKeyChainVal(child, target[child.key()], scalar);
            else
                return false;
        }
        return result;
    }

    // loop recursively through seq
    if (source.is_seq()) {
        bool result = true;

        for (std::size_t i = 0; i < source.num_children() && result == true; ++i) {
            result = getNodeFromKeyChainVal(source[i], target[i], scalar);
        }
        return result;
    }

    return false;
}
