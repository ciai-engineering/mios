#pragma once


#include "mongocxx/client.hpp"
#include "mongocxx/instance.hpp"
#include "mongocxx/database.hpp"
#include "mongocxx/collection.hpp"

#include "nlohmann/json.hpp"

#include <mutex>
#include <set>
#include <string>
#include <map>

namespace mios {

class MongodbClient{
public:
    MongodbClient(const std::string& database, unsigned port=27017,
                  bool connect_immediately=true);

    // The legacy application connects in the constructor. ROS-only Core
    // startup may defer the same blocking connection until Core::initialize.
    bool connect();

    bool read_document(const std::string& name, const std::string& collection, nlohmann::json& descr);
    bool read_documents(const std::string& collection,std::set<nlohmann::json>& docs);
    bool write_document(const std::string& name, const std::string& collection, const nlohmann::json &descr, bool overwrite);
    bool write_documents(const std::string& collection, const std::set<nlohmann::json>& docs, bool overwrite);
    bool write_large_document(const std::string& collection, const nlohmann::json &descr);
    bool make_document_consistent(const std::string& name, std::string collection, const nlohmann::json &template_json);
    bool health_check() const;

private:
    mongocxx::instance m_instance;
    mongocxx::client m_client;
    mongocxx::database m_mongodb;
    std::map<std::string,mongocxx::collection> m_collections;

    std::string m_database_name;
    unsigned m_database_port;
    bool m_connected{false};

    std::mutex m_mutex_db_access;
};

}
