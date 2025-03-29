#include "iter.h"

HexMapIter::operator Ref<hex_bind::HexMapIter>() const {
    return Ref<hex_bind::HexMapIter>(memnew(hex_bind::HexMapIter(*this)));
}

void hex_bind::HexMapIter::_bind_methods() {
    ClassDB::bind_method(
            D_METHOD("_iter_init", "_arg"), &hex_bind::HexMapIter::_iter_init);
    ClassDB::bind_method(
            D_METHOD("_iter_next", "_arg"), &hex_bind::HexMapIter::_iter_next);
    ClassDB::bind_method(
            D_METHOD("_iter_get", "_arg"), &hex_bind::HexMapIter::_iter_get);
}

// Godot custom iterator functions
bool hex_bind::HexMapIter::_iter_init(Variant _arg) {
    return iter->_iter_init();
}

bool hex_bind::HexMapIter::_iter_next(Variant _arg) {
    return iter->_iter_next();
}

Ref<hex_bind::HexMapCellId> hex_bind::HexMapIter::_iter_get(Variant _arg) {
    return iter->_iter_get();
}

String hex_bind::HexMapIter::_to_string() const { return *iter; };
