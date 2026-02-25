#pragma once

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace nexdyndiff::scene::detail
{
	inline std::string ToLower(const std::string& value)
	{
		std::string out = value;
		std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) { return (char)std::tolower(c); });
		return out;
	}

	inline std::string Trim(const std::string& value)
	{
		size_t begin = 0;
		while (begin < value.size() && std::isspace((unsigned char)value[begin])) ++begin;
		size_t end = value.size();
		while (end > begin && std::isspace((unsigned char)value[end - 1])) --end;
		return value.substr(begin, end - begin);
	}

	inline bool ParseDouble(const std::string& value, double& out)
	{
		char* end_ptr = nullptr;
		errno = 0;
		out = std::strtod(value.c_str(), &end_ptr);
		if (errno != 0 || end_ptr == value.c_str()) return false;
		while (*end_ptr != '\0' && std::isspace((unsigned char)*end_ptr)) ++end_ptr;
		return *end_ptr == '\0';
	}

	inline bool ParseInt(const std::string& value, int& out)
	{
		char* end_ptr = nullptr;
		errno = 0;
		const long parsed = std::strtol(value.c_str(), &end_ptr, 10);
		if (errno != 0 || end_ptr == value.c_str()) return false;
		while (*end_ptr != '\0' && std::isspace((unsigned char)*end_ptr)) ++end_ptr;
		if (*end_ptr != '\0') return false;
		out = (int)parsed;
		return true;
	}

	inline bool ParseBool(const std::string& value, bool& out)
	{
		const std::string lowered = ToLower(Trim(value));
		if (lowered == "true" || lowered == "1" || lowered == "yes") {
			out = true;
			return true;
		}
		if (lowered == "false" || lowered == "0" || lowered == "no") {
			out = false;
			return true;
		}
		return false;
	}

	inline std::vector<std::string> SplitTokens(const std::string& input)
	{
		std::string normalized = input;
		for (char& c : normalized) {
			if (c == ',' || c == ';') c = ' ';
		}
		std::istringstream stream(normalized);
		std::vector<std::string> out;
		std::string token;
		while (stream >> token) out.push_back(token);
		return out;
	}

	struct JsonValue
	{
		enum class Type { Null, Boolean, Number, String, Array, Object };
		Type type = Type::Null;
		bool b = false;
		double n = 0.0;
		std::string s;
		std::vector<JsonValue> a;
		std::unordered_map<std::string, JsonValue> o;

		bool IsObject() const { return type == Type::Object; }
		bool IsArray() const { return type == Type::Array; }
		bool IsString() const { return type == Type::String; }
		bool IsNumber() const { return type == Type::Number; }
		bool IsBool() const { return type == Type::Boolean; }

		const JsonValue* Find(const std::string& key) const
		{
			auto it = o.find(key);
			return it == o.end() ? nullptr : &it->second;
		}
	};

	class JsonParser
	{
	public:
		explicit JsonParser(const std::string& text) : m_text(text) {}

		bool Parse(JsonValue& out, std::string& err)
		{
			SkipWs();
			if (!ParseValue(out, err)) return false;
			SkipWs();
			if (m_pos != m_text.size()) {
				err = "Unexpected trailing content in JSON";
				return false;
			}
			return true;
		}

	private:
		const std::string& m_text;
		size_t m_pos = 0;

		void SkipWs()
		{
			while (m_pos < m_text.size() && std::isspace((unsigned char)m_text[m_pos])) ++m_pos;
		}

		bool ParseValue(JsonValue& out, std::string& err)
		{
			SkipWs();
			if (m_pos >= m_text.size()) {
				err = "Unexpected end of JSON";
				return false;
			}
			const char c = m_text[m_pos];
			if (c == '{') return ParseObject(out, err);
			if (c == '[') return ParseArray(out, err);
			if (c == '"') {
				out.type = JsonValue::Type::String;
				return ParseString(out.s, err);
			}
			if (c == 't' || c == 'f') return ParseBoolValue(out, err);
			if (c == 'n') return ParseNullValue(out, err);
			return ParseNumberValue(out, err);
		}

		bool ParseObject(JsonValue& out, std::string& err)
		{
			++m_pos;
			SkipWs();
			out.type = JsonValue::Type::Object;
			out.o.clear();
			if (m_pos < m_text.size() && m_text[m_pos] == '}') {
				++m_pos;
				return true;
			}
			while (m_pos < m_text.size()) {
				std::string key;
				SkipWs();
				if (!ParseString(key, err)) return false;
				SkipWs();
				if (m_pos >= m_text.size() || m_text[m_pos] != ':') {
					err = "Expected ':' in JSON object";
					return false;
				}
				++m_pos;
				SkipWs();
				JsonValue value;
				if (!ParseValue(value, err)) return false;
				out.o[key] = value;
				SkipWs();
				if (m_pos >= m_text.size()) {
					err = "Unexpected end of JSON object";
					return false;
				}
				if (m_text[m_pos] == ',') {
					++m_pos;
					continue;
				}
				if (m_text[m_pos] == '}') {
					++m_pos;
					return true;
				}
				err = "Expected ',' or '}' in JSON object";
				return false;
			}
			err = "Unexpected end of JSON object";
			return false;
		}

		bool ParseArray(JsonValue& out, std::string& err)
		{
			++m_pos;
			SkipWs();
			out.type = JsonValue::Type::Array;
			out.a.clear();
			if (m_pos < m_text.size() && m_text[m_pos] == ']') {
				++m_pos;
				return true;
			}
			while (m_pos < m_text.size()) {
				JsonValue value;
				if (!ParseValue(value, err)) return false;
				out.a.push_back(value);
				SkipWs();
				if (m_pos >= m_text.size()) {
					err = "Unexpected end of JSON array";
					return false;
				}
				if (m_text[m_pos] == ',') {
					++m_pos;
					continue;
				}
				if (m_text[m_pos] == ']') {
					++m_pos;
					return true;
				}
				err = "Expected ',' or ']' in JSON array";
				return false;
			}
			err = "Unexpected end of JSON array";
			return false;
		}

		bool ParseString(std::string& out, std::string& err)
		{
			if (m_pos >= m_text.size() || m_text[m_pos] != '"') {
				err = "Expected string";
				return false;
			}
			++m_pos;
			out.clear();
			while (m_pos < m_text.size()) {
				const char c = m_text[m_pos++];
				if (c == '"') return true;
				if (c == '\\') {
					if (m_pos >= m_text.size()) {
						err = "Invalid escape in JSON string";
						return false;
					}
					const char e = m_text[m_pos++];
					switch (e) {
					case '"': out.push_back('"'); break;
					case '\\': out.push_back('\\'); break;
					case '/': out.push_back('/'); break;
					case 'b': out.push_back('\b'); break;
					case 'f': out.push_back('\f'); break;
					case 'n': out.push_back('\n'); break;
					case 'r': out.push_back('\r'); break;
					case 't': out.push_back('\t'); break;
					default:
						err = "Unsupported JSON escape";
						return false;
					}
					continue;
				}
				out.push_back(c);
			}
			err = "Unterminated JSON string";
			return false;
		}

		bool ParseBoolValue(JsonValue& out, std::string& err)
		{
			if (m_text.compare(m_pos, 4, "true") == 0) {
				m_pos += 4;
				out.type = JsonValue::Type::Boolean;
				out.b = true;
				return true;
			}
			if (m_text.compare(m_pos, 5, "false") == 0) {
				m_pos += 5;
				out.type = JsonValue::Type::Boolean;
				out.b = false;
				return true;
			}
			err = "Invalid JSON boolean";
			return false;
		}

		bool ParseNullValue(JsonValue& out, std::string& err)
		{
			if (m_text.compare(m_pos, 4, "null") != 0) {
				err = "Invalid JSON null";
				return false;
			}
			m_pos += 4;
			out.type = JsonValue::Type::Null;
			return true;
		}

		bool ParseNumberValue(JsonValue& out, std::string& err)
		{
			char* end_ptr = nullptr;
			errno = 0;
			const double parsed = std::strtod(m_text.c_str() + m_pos, &end_ptr);
			if (errno != 0 || end_ptr == m_text.c_str() + m_pos) {
				err = "Invalid JSON number";
				return false;
			}
			m_pos = (size_t)(end_ptr - m_text.c_str());
			out.type = JsonValue::Type::Number;
			out.n = parsed;
			return true;
		}
	};

	struct XmlNode
	{
		std::string name;
		std::unordered_map<std::string, std::string> attributes;
		std::vector<XmlNode> children;
		std::string text;

		const XmlNode* FindChild(const std::string& child_name) const
		{
			for (const XmlNode& child : children) if (child.name == child_name) return &child;
			return nullptr;
		}

		std::vector<const XmlNode*> FindChildren(const std::string& child_name) const
		{
			std::vector<const XmlNode*> out;
			for (const XmlNode& child : children) if (child.name == child_name) out.push_back(&child);
			return out;
		}

		std::optional<std::string> Attribute(const std::string& key) const
		{
			auto it = attributes.find(key);
			return it == attributes.end() ? std::nullopt : std::optional<std::string>(it->second);
		}
	};

	class XmlParser
	{
	public:
		explicit XmlParser(const std::string& text) : m_text(text) {}

		bool Parse(XmlNode& out, std::string& err)
		{
			SkipWs();
			while (SkipProlog(err)) SkipWs();
			if (!ParseElement(out, err)) return false;
			SkipWs();
			if (m_pos != m_text.size()) {
				err = "Unexpected trailing content in XML";
				return false;
			}
			return true;
		}

	private:
		const std::string& m_text;
		size_t m_pos = 0;

		void SkipWs() { while (m_pos < m_text.size() && std::isspace((unsigned char)m_text[m_pos])) ++m_pos; }

		bool StartsWith(const std::string& value) const
		{
			return m_pos + value.size() <= m_text.size() && m_text.compare(m_pos, value.size(), value) == 0;
		}

		bool SkipProlog(std::string& err)
		{
			if (StartsWith("<?")) {
				size_t end = m_text.find("?>", m_pos + 2);
				if (end == std::string::npos) {
					err = "Unterminated XML declaration";
					return false;
				}
				m_pos = end + 2;
				return true;
			}
			if (StartsWith("<!--")) {
				size_t end = m_text.find("-->", m_pos + 4);
				if (end == std::string::npos) {
					err = "Unterminated XML comment";
					return false;
				}
				m_pos = end + 3;
				return true;
			}
			return false;
		}

		bool ParseName(std::string& out)
		{
			size_t begin = m_pos;
			while (m_pos < m_text.size()) {
				char c = m_text[m_pos];
				if (std::isalnum((unsigned char)c) || c == '_' || c == '-' || c == ':' || c == '.') ++m_pos;
				else break;
			}
			if (m_pos == begin) return false;
			out = m_text.substr(begin, m_pos - begin);
			return true;
		}

		bool ParseAttributeValue(std::string& out, std::string& err)
		{
			if (m_pos >= m_text.size()) {
				err = "Unexpected end of XML attribute";
				return false;
			}
			const char quote = m_text[m_pos];
			if (quote != '"' && quote != '\'') {
				err = "Expected quote in XML attribute";
				return false;
			}
			++m_pos;
			size_t begin = m_pos;
			while (m_pos < m_text.size() && m_text[m_pos] != quote) ++m_pos;
			if (m_pos >= m_text.size()) {
				err = "Unterminated XML attribute";
				return false;
			}
			out = m_text.substr(begin, m_pos - begin);
			++m_pos;
			return true;
		}

		bool ParseElement(XmlNode& out, std::string& err)
		{
			if (m_pos >= m_text.size() || m_text[m_pos] != '<') {
				err = "Expected '<' in XML";
				return false;
			}
			++m_pos;
			if (!ParseName(out.name)) {
				err = "Invalid XML node name";
				return false;
			}
			out.attributes.clear();
			out.children.clear();
			out.text.clear();

			while (true) {
				SkipWs();
				if (m_pos >= m_text.size()) {
					err = "Unexpected end of XML element";
					return false;
				}
				if (m_text[m_pos] == '/') {
					++m_pos;
					if (m_pos >= m_text.size() || m_text[m_pos] != '>') {
						err = "Invalid self-closing XML tag";
						return false;
					}
					++m_pos;
					return true;
				}
				if (m_text[m_pos] == '>') {
					++m_pos;
					break;
				}
				std::string attr_name;
				if (!ParseName(attr_name)) {
					err = "Invalid XML attribute name";
					return false;
				}
				SkipWs();
				if (m_pos >= m_text.size() || m_text[m_pos] != '=') {
					err = "Expected '=' in XML attribute";
					return false;
				}
				++m_pos;
				SkipWs();
				std::string attr_value;
				if (!ParseAttributeValue(attr_value, err)) return false;
				out.attributes[attr_name] = attr_value;
			}

			while (true) {
				SkipWs();
				if (m_pos >= m_text.size()) {
					err = "Unexpected end while parsing XML content";
					return false;
				}
				if (StartsWith("</")) {
					m_pos += 2;
					std::string closing_name;
					if (!ParseName(closing_name)) {
						err = "Invalid XML closing tag";
						return false;
					}
					SkipWs();
					if (m_pos >= m_text.size() || m_text[m_pos] != '>') {
						err = "Expected '>' in closing tag";
						return false;
					}
					++m_pos;
					if (closing_name != out.name) {
						err = "Mismatched XML closing tag";
						return false;
					}
					return true;
				}
				if (StartsWith("<!--")) {
					if (!SkipProlog(err)) return false;
					continue;
				}
				if (m_text[m_pos] == '<') {
					XmlNode child;
					if (!ParseElement(child, err)) return false;
					out.children.push_back(child);
					continue;
				}
				size_t begin = m_pos;
				while (m_pos < m_text.size() && m_text[m_pos] != '<') ++m_pos;
				const std::string content = Trim(m_text.substr(begin, m_pos - begin));
				if (!content.empty()) {
					if (!out.text.empty()) out.text.push_back(' ');
					out.text += content;
				}
			}
		}
	};
}
