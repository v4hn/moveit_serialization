#include <moveit_serialization/ryml/utils.h>

/* Given */
bool c4::yml::getValFromKeyChain(const c4::yml::NodeRef& source, const c4::yml::NodeRef& target,
                                 c4::yml::NodeRef& scalar)
{
    if (source.type() == c4::yml::VAL || source.type() == c4::yml::KEYVAL)
        if (source.type() != target.type())
            return false;

    if (source.type() == (c4::yml::VAL || c4::yml::KEYVAL)) {
        // clone value
        scalar.tree()->merge_with(target.tree());
        return true;
    } else if (source.type() == c4::yml::NONE)
        return false;
    else if (source.type() == (c4::yml::MAP || c4::yml::SEQ)) {
        if (source.num_children() > target.num_children())
            return false;
    }

    // loop recursively through map
    if (source.is_map()) {
        bool result = true;

        for (c4::yml::NodeRef const& child : source.children()) {
            if (target.has_child(child.key()))
                result = getValFromKeyChain(child, target[child.key()], scalar);
            else
                return false;
        }
        return result;
    }

    // loop recursively through seq
    if (source.is_seq()) {
        bool result = true;

        for (std::size_t i = 0; i < source.num_children() && result == true; ++i) {
            result = getValFromKeyChain(source[i], target[i], scalar);
        }
        return result;
    }

    return false;
}
