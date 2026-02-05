## Tech stack

Define your technical stack below. This serves as a reference for all team members and helps maintain consistency across the project.

### Framework & Runtime
- **Application Framework:** Dioxus
- **Language/Runtime:** Rust
- **Package Manager:** cargo

### Frontend
- **Rust Frontend Framework:** Dioxus
- **CSS Framework:** Tailwind CSS
- **UI Components:** (optional -- no dominant Rust-native UI library) or Dioxus-specific component crates

### Database & Storage
- **Database:** PostgreSQL, SQLite
- **ORM/Query Builder:** SeaORM, SQLx, Diesel
- **Caching:** [e.g., Redis, Memcached]

### Testing & Quality
- **Test Framework:** cargo test
- **Integration/HTTP Testing: reqwest, httpmock, axum-test
- **Linting/Formatting:** clippy and rustfmt

### Deployment & Infrastructure
- **Hosting:** Shuttle, Digital Ocean, Fly.io, Railway, Vercel, Heroku
- **CI/CD:** GitHub Actions

### Third-Party Services
- **Authentication:** JWT via jsonwebtoken, OAuth2 via oauth2 crate
- **Email:** lettre
- **Monitoring:** Sentry (Rust SDK), Datadog via OpenTelemetry crates
