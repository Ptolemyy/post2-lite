#pragma once

#include <map>
#include <string>
#include <vector>

namespace post2::core {

class JsonValue {
public:
    enum class Type {
        Null,
        Bool,
        Number,
        String,
        Array,
        Object,
    };

    using Array = std::vector<JsonValue>;
    using Object = std::map<std::string, JsonValue>;

    Type type = Type::Null;
    bool bool_value = false;
    double number_value = 0.0;
    std::string string_value;
    Array array_value;
    Object object_value;

    static JsonValue null();
    static JsonValue boolean(bool value);
    static JsonValue number(double value);
    static JsonValue string(std::string value);
    static JsonValue array(Array value = {});
    static JsonValue object(Object value = {});

    bool is_null() const;
    bool is_bool() const;
    bool is_number() const;
    bool is_string() const;
    bool is_array() const;
    bool is_object() const;
};

bool parse_json(const std::string& text, JsonValue* value, std::string* error);
std::string json_to_string(const JsonValue& value, bool pretty = true);
const JsonValue* find_member(const JsonValue& object, const std::string& key);

} // namespace post2::core
