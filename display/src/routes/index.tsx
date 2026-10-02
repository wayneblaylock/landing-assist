import { createFileRoute } from "@tanstack/react-router";
import { HorizonDisplay } from "@/components/horizon-display";

export const Route = createFileRoute("/")({ component: Home });

function Home() {
  return <HorizonDisplay />;
}
