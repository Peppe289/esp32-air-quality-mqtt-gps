FROM node:20-alpine3.22 AS build-stage
WORKDIR /app
COPY package*.json ./
RUN npm install
COPY . .
# generate static files in dist folder
RUN npm run build

# Move react build to nginx html folder
FROM nginx:stable-alpine
COPY --from=build-stage /app/dist /usr/share/nginx/html
COPY nginx.conf /etc/nginx/conf.d/default.conf
EXPOSE 5173
CMD ["nginx", "-g", "daemon off;"]