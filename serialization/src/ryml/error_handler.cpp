#include <moveit_serialization/ryml/error_handler.h>

namespace moveit_serialization {

// https://pabloariasal.github.io/2020/01/02/static-variable-initialization/#solving-the-static-initialization-order-fiasco
auto& registerErrorHandler()
{
    static auto err = ErrorHandler();
    c4::yml::set_callbacks(err.callbacks());

    return err;
}

namespace {

auto ERROR_HANDLER = registerErrorHandler();

}  // namespace

}  // namespace moveit_serialization
