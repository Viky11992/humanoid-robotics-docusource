---
description: Monitor system health and performance of backend services
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

Given the user arguments, implement system monitoring for the Physical AI & Humanoid Robotics book project:

1. **Parse arguments** to determine monitoring mode:
   - `--service <name>`: Check specific service (backend, qdrant, cohere, postgres)
   - `--all` or no args: Check overall system status
   - `--metrics`: Show detailed performance metrics
   - `--alerts`: Show active alerts and issues
   - Default: Show overall system status

2. **Monitoring Components**:
   - Backend API health (FastAPI service)
   - Qdrant vector database connectivity
   - Cohere API connectivity and rate limits
   - PostgreSQL database connectivity
   - API response times and performance
   - Resource utilization (CPU, memory, disk)

3. **Health Checks**:
   - API endpoint availability
   - Database connection status
   - External service connectivity (Cohere, Qdrant cloud)
   - Response time measurements
   - Error rate monitoring
   - Resource usage tracking

4. **Integration with Existing System**:
   - Use existing health check endpoints from `backend/src/api/health_router.py`
   - Access database status through existing models
   - Check external services using existing configurations
   - Monitor API performance using existing logging

5. **Monitoring Process**:
   - Perform connectivity tests for each component
   - Measure response times and performance metrics
   - Check API rate limits and usage quotas
   - Validate database connections and query performance
   - Assess system capacity and resource availability

6. **Output**:
   - Overall system health status (OK, Warning, Critical)
   - Individual component statuses
   - Performance metrics and response times
   - Resource utilization statistics
   - Alerts and recommendations for improvement

## Implementation Notes

- Use existing health check infrastructure from the backend
- Follow the same monitoring patterns as existing system
- Integrate with existing logging and error reporting
- Provide actionable insights for system optimization
- Maintain compatibility with existing deployment architecture

---