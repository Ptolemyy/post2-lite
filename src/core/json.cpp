#include "post2/core/json.hpp"

#include <cctype>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace post2::core {

JsonValue JsonValue::null()
{
    return {};
}

JsonValue JsonValue::boolean(bool value)
{
    JsonValue json;
    json.type = Type::Bool;
    json.bool_value = value;
    return json;
}

JsonValue JsonValue::number(double value)
{
    JsonValue json;
    json.type = Type::Number;
    json.number_value = value;
    return json;
}

JsonValue JsonValue::string(std::string value)
{
    JsonValue json;
    json.type = Type::String;
    json.string_value = std::move(value);
    return json;
}

JsonValue JsonValue::array(Array value)
{
    JsonValue json;
    json.type = Type::Array;
    json.array_value = std::move(value);
    return json;
}

JsonValue JsonValue::object(Object value)
{
    JsonValue json;
    json.type = Type::Object;
    json.object_value = std::move(value);
    return json;
}

bool JsonValue::is_null() const { return type == Type::Null; }
bool JsonValue::is_bool() const { return type == Type::Bool; }
bool JsonValue::is_number() const { return type == Type::Number; }
bool JsonValue::is_string() const { return type == Type::String; }
bool JsonValue::is_array() const { return type == Type::Array; }
bool JsonValue::is_object() const { return type == Type::Object; }

namespace {

class JsonParser {
public:
    explicit JsonParser(const std::string& text)
        : text_(text)
    {
    }

    JsonValue parse()
    {
        skip_space();
        JsonValue value = parse_value();
        skip_space();
        if (position_ != text_.size()) {
            fail("unexpected trailing characters");
        }
        return value;
    }

private:
    char peek() const
    {
        return position_ < text_.size() ? text_[position_] : '\0';
    }

    char get()
    {
        if (position_ >= text_.size()) {
            fail("unexpected end of JSON");
        }
        return text_[position_++];
    }

    void skip_space()
    {
        while (position_ < text_.size() && std::isspace(static_cast<unsigned char>(text_[position_])) != 0) {
            ++position_;
        }
    }

    bool consume(char ch)
    {
        skip_space();
        if (peek() != ch) {
            return false;
        }
        ++position_;
        return true;
    }

    void expect_literal(const char* literal)
    {
        for (const char* cursor = literal; *cursor != '\0'; ++cursor) {
            if (get() != *cursor) {
                fail(std::string("expected ") + literal);
            }
        }
    }

    JsonValue parse_value()
    {
        skip_space();
        const char ch = peek();
        if (ch == '{') {
            return parse_object();
        }
        if (ch == '[') {
            return parse_array();
        }
        if (ch == '"') {
            return JsonValue::string(parse_string());
        }
        if (ch == 't') {
            expect_literal("true");
            return JsonValue::boolean(true);
        }
        if (ch == 'f') {
            expect_literal("false");
            return JsonValue::boolean(false);
        }
        if (ch == 'n') {
            expect_literal("null");
            return JsonValue::null();
        }
        if (ch == '-' || std::isdigit(static_cast<unsigned char>(ch)) != 0) {
            return parse_number();
        }
        fail("expected JSON value");
    }

    JsonValue parse_object()
    {
        get();
        JsonValue::Object object;
        skip_space();
        if (consume('}')) {
            return JsonValue::object(std::move(object));
        }

        for (;;) {
            skip_space();
            if (peek() != '"') {
                fail("expected object key string");
            }
            std::string key = parse_string();
            if (!consume(':')) {
                fail("expected ':' after object key");
            }
            object[std::move(key)] = parse_value();
            if (consume('}')) {
                break;
            }
            if (!consume(',')) {
                fail("expected ',' or '}' in object");
            }
        }
        return JsonValue::object(std::move(object));
    }

    JsonValue parse_array()
    {
        get();
        JsonValue::Array array;
        skip_space();
        if (consume(']')) {
            return JsonValue::array(std::move(array));
        }

        for (;;) {
            array.push_back(parse_value());
            if (consume(']')) {
                break;
            }
            if (!consume(',')) {
                fail("expected ',' or ']' in array");
            }
        }
        return JsonValue::array(std::move(array));
    }

    std::string parse_string()
    {
        if (get() != '"') {
            fail("expected string");
        }

        std::string result;
        for (;;) {
            const char ch = get();
            if (ch == '"') {
                break;
            }
            if (ch == '\\') {
                const char escaped = get();
                switch (escaped) {
                case '"':
                case '\\':
                case '/':
                    result.push_back(escaped);
                    break;
                case 'b':
                    result.push_back('\b');
                    break;
                case 'f':
                    result.push_back('\f');
                    break;
                case 'n':
                    result.push_back('\n');
                    break;
                case 'r':
                    result.push_back('\r');
                    break;
                case 't':
                    result.push_back('\t');
                    break;
                default:
                    fail("unsupported string escape");
                }
            } else {
                result.push_back(ch);
            }
        }
        return result;
    }

    JsonValue parse_number()
    {
        const std::size_t start = position_;
        if (peek() == '-') {
            ++position_;
        }
        while (std::isdigit(static_cast<unsigned char>(peek())) != 0) {
            ++position_;
        }
        if (peek() == '.') {
            ++position_;
            while (std::isdigit(static_cast<unsigned char>(peek())) != 0) {
                ++position_;
            }
        }
        if (peek() == 'e' || peek() == 'E') {
            ++position_;
            if (peek() == '+' || peek() == '-') {
                ++position_;
            }
            while (std::isdigit(static_cast<unsigned char>(peek())) != 0) {
                ++position_;
            }
        }

        try {
            return JsonValue::number(std::stod(text_.substr(start, position_ - start)));
        } catch (const std::exception&) {
            fail("invalid number");
        }
    }

    [[noreturn]] void fail(const std::string& message) const
    {
        throw std::runtime_error(message + " at byte " + std::to_string(position_));
    }

    const std::string& text_;
    std::size_t position_ = 0;
};

void indent(std::ostringstream& output, int level)
{
    for (int i = 0; i < level; ++i) {
        output << "  ";
    }
}

void write_escaped_string(std::ostringstream& output, const std::string& value)
{
    output << '"';
    for (const char ch : value) {
        switch (ch) {
        case '"':
            output << "\\\"";
            break;
        case '\\':
            output << "\\\\";
            break;
        case '\b':
            output << "\\b";
            break;
        case '\f':
            output << "\\f";
            break;
        case '\n':
            output << "\\n";
            break;
        case '\r':
            output << "\\r";
            break;
        case '\t':
            output << "\\t";
            break;
        default:
            output << ch;
            break;
        }
    }
    output << '"';
}

void write_json(std::ostringstream& output, const JsonValue& value, bool pretty, int level)
{
    switch (value.type) {
    case JsonValue::Type::Null:
        output << "null";
        break;
    case JsonValue::Type::Bool:
        output << (value.bool_value ? "true" : "false");
        break;
    case JsonValue::Type::Number:
        output << std::setprecision(17) << value.number_value;
        break;
    case JsonValue::Type::String:
        write_escaped_string(output, value.string_value);
        break;
    case JsonValue::Type::Array:
        output << '[';
        for (std::size_t i = 0; i < value.array_value.size(); ++i) {
            if (i > 0) {
                output << ',';
            }
            if (pretty) {
                output << '\n';
                indent(output, level + 1);
            }
            write_json(output, value.array_value[i], pretty, level + 1);
        }
        if (pretty && !value.array_value.empty()) {
            output << '\n';
            indent(output, level);
        }
        output << ']';
        break;
    case JsonValue::Type::Object:
        output << '{';
        {
            std::size_t i = 0;
            for (const auto& [key, member] : value.object_value) {
                if (i++ > 0) {
                    output << ',';
                }
                if (pretty) {
                    output << '\n';
                    indent(output, level + 1);
                }
                write_escaped_string(output, key);
                output << (pretty ? ": " : ":");
                write_json(output, member, pretty, level + 1);
            }
        }
        if (pretty && !value.object_value.empty()) {
            output << '\n';
            indent(output, level);
        }
        output << '}';
        break;
    }
}

} // namespace

bool parse_json(const std::string& text, JsonValue* value, std::string* error)
{
    try {
        *value = JsonParser(text).parse();
        return true;
    } catch (const std::exception& ex) {
        if (error) {
            *error = ex.what();
        }
        return false;
    }
}

std::string json_to_string(const JsonValue& value, bool pretty)
{
    std::ostringstream output;
    write_json(output, value, pretty, 0);
    output << '\n';
    return output.str();
}

const JsonValue* find_member(const JsonValue& object, const std::string& key)
{
    if (!object.is_object()) {
        return nullptr;
    }
    const auto it = object.object_value.find(key);
    return it == object.object_value.end() ? nullptr : &it->second;
}

} // namespace post2::core
